package armplanning

import (
	"context"
	"errors"
	"fmt"

	vizClient "github.com/viam-labs/motion-tools/client/client"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// CheckPlanFromRequest checks if obstacles intersect the trajectory of the frame following the plan.
// This is a convenience wrapper around CheckPlan that extracts the necessary information from a PlanRequest.
// It checks for:
// 1. Self-collision (moving frames against other parts of the frame system)
// 2. World collision (moving frames against entities in worldstate)
// 3. Collisions not just at waypoints but interpolated along segments (e.g., when moving from a→b→c)
// Returns all collisions found as a single error.
// The moving frames are automatically detected by analyzing which frames have changing inputs in the trajectory.
// Collisions that exist at the start of the trajectory (index 0) are automatically allowed throughout the plan.
func CheckPlanFromRequest(
	ctx context.Context,
	req *PlanRequest,
	plan motionplan.Plan,
) ([]referenceframe.Input, error) {
	if req == nil {
		return nil, errors.New("plan request cannot be nil")
	}
	if req.FrameSystem == nil {
		return nil, errors.New("plan request must have a frame system")
	}
	if req.WorldState == nil {
		return nil, errors.New("plan request must have a world state")
	}

	return CheckPlan(ctx, req.WorldState, req.FrameSystem, plan)
}

// CheckPlan checks if obstacles intersect the trajectory of the frame following the plan.
// It checks for:
// 1. Self-collision (moving frames against other parts of the frame system)
// 2. World collision (moving frames against entities in worldstate)
// 3. Collisions not just at waypoints but interpolated along segments (e.g., when moving from a→b→c)
// Returns all collisions found as a single error.
// The moving frames are automatically detected by analyzing which frames have changing inputs in the trajectory.
// Collisions that exist at the start of the trajectory (index 0) are automatically allowed throughout the plan.
func CheckPlan(
	ctx context.Context,
	worldstate *referenceframe.WorldState,
	fs *referenceframe.FrameSystem,
	plan motionplan.Plan,
) ([]referenceframe.Input, error) {
	trajectory := plan.Trajectory()

	if len(trajectory) < 1 {
		return nil, errors.New("cannot check an empty plan")
	}

	// Convert trajectory to linear inputs for easier handling
	var linearTrajectory []*referenceframe.LinearInputs
	for _, frameSystemInputs := range trajectory {
		linearInputs := frameSystemInputs.ToLinearInputs()
		linearTrajectory = append(linearTrajectory, linearInputs)
	}

	// Get the frame system geometries at the start configuration
	seedMap := linearTrajectory[0]
	frameSystemGeometries, err := referenceframe.FrameSystemGeometriesLinearInputs(fs, seedMap)
	if err != nil {
		return nil, err
	}

	// Automatically detect which frames are moving by analyzing the trajectory
	movingFrames := detectMovingFrames(trajectory, fs)

	// Separate moving geometries from static geometries based on detected moving frames
	var movingGeometries []spatialmath.Geometry
	var staticGeometries []spatialmath.Geometry

	for _, geoms := range frameSystemGeometries {
		for _, geom := range geoms.Geometries() {
			// Check if this geometry belongs to any of the moving frames
			if belongsToMovingFrame(geom.Label(), movingFrames) {
				movingGeometries = append(movingGeometries, geom)
			} else {
				staticGeometries = append(staticGeometries, geom)
			}
		}
	}

	// Detect collisions at index 0 (start configuration) to use as allowed collisions
	// This prevents the checker from failing on collisions that already exist at the start
	logger := logging.NewLogger("CheckPlan")
	collisionBufferMM := NewBasicPlannerOptions().CollisionBufferMM

	// Get world geometries at the start configuration
	obstaclesInFrame, err := worldstate.ObstaclesInWorldFrame(fs, seedMap.ToFrameSystemInputs())
	if err != nil {
		return nil, err
	}
	worldGeometries := obstaclesInFrame.Geometries()

	// Detect all collisions at the start configuration
	var allowedCollisions []*motionplan.Collision

	// Check moving vs world collisions
	if len(worldGeometries) > 0 && len(movingGeometries) > 0 {
		worldCG, err := motionplan.NewCollisionGraphFromGeometries(
			fs, movingGeometries, worldGeometries, nil, true, collisionBufferMM,
		)
		if err != nil {
			return nil, err
		}
		for _, col := range worldCG.Collisions(collisionBufferMM) {
			allowedCollisions = append(allowedCollisions, &col)
		}
	}

	// Check moving vs static collisions
	if len(staticGeometries) > 0 && len(movingGeometries) > 0 {
		staticCG, err := motionplan.NewCollisionGraphFromGeometries(
			fs, movingGeometries, staticGeometries, nil, true, collisionBufferMM,
		)
		if err != nil {
			return nil, err
		}
		for _, col := range staticCG.Collisions(collisionBufferMM) {
			allowedCollisions = append(allowedCollisions, &col)
		}
	}

	// Check self collisions (moving vs moving)
	if len(movingGeometries) > 1 {
		selfCG, err := motionplan.NewCollisionGraphFromGeometries(
			fs, movingGeometries, nil, nil, true, collisionBufferMM,
		)
		if err != nil {
			return nil, err
		}
		for _, col := range selfCG.Collisions(collisionBufferMM) {
			allowedCollisions = append(allowedCollisions, &col)
		}
	}

	// Create collision constraints with allowed collisions
	collisionConstraints, err := motionplan.CreateAllCollisionConstraints(
		fs,
		movingGeometries,
		staticGeometries,
		worldGeometries,
		allowedCollisions,
		collisionBufferMM,
	)
	if err != nil {
		return nil, err
	}

	// Create constraint checker and set the collision constraints
	checker := motionplan.NewEmptyConstraintChecker(logger)
	checker.SetCollisionConstraints(collisionConstraints)

	// Resolution of interpolation
	const resolution = 1

	// Check each segment in the trajectory with interpolation
	for i := 0; i < len(linearTrajectory)-1; i++ {
		segment := &motionplan.SegmentFS{
			StartConfiguration: linearTrajectory[i],
			EndConfiguration:   linearTrajectory[i+1],
			FS:                 fs,
		}

		// Interpolate and check the segment
		interpolatedConfigurations, err := motionplan.InterpolateSegmentFS(segment, resolution)
		if err != nil {
			return nil, err
		}

		// Check each interpolated configuration
		// Skip the first configuration (j=0) for the very first segment (i=0) as it's the start configuration,
		// which may already be in collision
		startIdx := 0
		if i == 0 {
			startIdx = 1
		}
		for j := startIdx; j < len(interpolatedConfigurations); j++ {
			interpConfig := interpolatedConfigurations[j]
			state := &motionplan.StateFS{
				FS:            fs,
				Configuration: interpConfig,
			}

			// Visualize all moving geometries at this configuration
			interpGeometries, err := referenceframe.FrameSystemGeometriesLinearInputs(fs, interpConfig)
			if err == nil {
				var movingGeomsToVisualize []spatialmath.Geometry
				for _, geomsInFrame := range interpGeometries {
					for _, geom := range geomsInFrame.Geometries() {
						if belongsToMovingFrame(geom.Label(), movingFrames) {
							movingGeomsToVisualize = append(movingGeomsToVisualize, geom)
						}
					}
				}
				if len(movingGeomsToVisualize) > 0 {
					colors := make([]string, len(movingGeomsToVisualize))
					for i := range colors {
						colors[i] = "orange"
					}
					vizClient.DrawGeometries(
						referenceframe.NewGeometriesInFrame("moving", movingGeomsToVisualize),
						colors,
					)
				}
			}

			if _, err := checker.CheckStateFSConstraints(ctx, state); err != nil {
				return interpConfig.GetLinearizedInputs(), fmt.Errorf("collision in segment %d at interpolation step %d (between waypoint %d and %d): %w", i, j, i, i+1, err)
			}
		}
	}

	return nil, nil
}

// detectMovingFrames analyzes the trajectory to determine which frames have changing inputs.
// Returns a set of frame names that are considered "moving".
func detectMovingFrames(trajectory motionplan.Trajectory, fs *referenceframe.FrameSystem) map[string]bool {
	movingFrames := make(map[string]bool)

	if len(trajectory) == 0 {
		return movingFrames
	}

	// First, collect all frames that appear in the trajectory
	framesInTrajectory := make(map[string]bool)
	for _, waypoint := range trajectory {
		for frameName := range waypoint {
			framesInTrajectory[frameName] = true
		}
	}

	// For each frame in the trajectory, check if its inputs change
	for frameName := range framesInTrajectory {
		if hasChangingInputs(frameName, trajectory) {
			movingFrames[frameName] = true
		}
	}

	// If no frames have changing inputs (e.g., trajectory with identical waypoints),
	// consider all frames in the trajectory as moving
	if len(movingFrames) == 0 {
		for frameName := range framesInTrajectory {
			movingFrames[frameName] = true
		}
	}

	return movingFrames
}

// hasChangingInputs checks if a frame's inputs change across the trajectory.
func hasChangingInputs(frameName string, trajectory motionplan.Trajectory) bool {
	if len(trajectory) < 2 {
		return false
	}

	// Get the first configuration for this frame
	firstInputs := trajectory[0][frameName]
	if firstInputs == nil {
		return false
	}

	// Compare with subsequent configurations
	for i := 1; i < len(trajectory); i++ {
		currentInputs := trajectory[i][frameName]
		if currentInputs == nil {
			continue
		}

		// Check if any input value differs
		if len(firstInputs) != len(currentInputs) {
			return true
		}

		for j := range firstInputs {
			// Input is a type alias for float64, so we can compare directly
			if firstInputs[j] != currentInputs[j] {
				return true
			}
		}
	}

	return false
}

// belongsToMovingFrame checks if a geometry label belongs to any of the moving frames.
// Geometries are typically labeled as "frameName:geometryName".
func belongsToMovingFrame(geometryLabel string, movingFrames map[string]bool) bool {
	for frameName := range movingFrames {
		prefix := frameName + ":"
		if len(geometryLabel) >= len(prefix) && geometryLabel[:len(prefix)] == prefix {
			return true
		}
	}
	return false
}
