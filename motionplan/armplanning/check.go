package armplanning

import (
	"context"
	"fmt"

	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// CheckPlan checks if obstacles intersect the trajectory of the frame following the plan.
// It checks for:
// 1. Self-collision (checkFrame against other parts of the frame system)
// 2. World collision (checkFrame against entities in worldstate)
// 3. Collisions not just at waypoints but interpolated along segments (e.g., when moving from a→b→c)
// Returns all collisions found as a single error.
func CheckPlan(
	ctx context.Context,
	checkFrame referenceframe.Frame,
	worldstate *referenceframe.WorldState,
	fs *referenceframe.FrameSystem,
	plan motionplan.Plan,
) error {
	trajectory := plan.Trajectory()

	// Validate that the plan has at least one element
	if len(trajectory) < 1 {
		return nil // empty plan, nothing to check
	}

	// Convert trajectory to linear inputs for easier handling
	var linearTrajectory []*referenceframe.LinearInputs
	for _, frameSystemInputs := range trajectory {
		linearInputs := frameSystemInputs.ToLinearInputs()
		linearTrajectory = append(linearTrajectory, linearInputs)
	}

	// Get the checkFrame's geometries to identify what's moving
	seedMap := linearTrajectory[0]
	frameSystemGeometries, err := referenceframe.FrameSystemGeometriesLinearInputs(fs, seedMap)
	if err != nil {
		return err
	}

	// Separate moving geometries (checkFrame) from static geometries (other frames)
	var movingGeometries []spatialmath.Geometry
	var staticGeometries []spatialmath.Geometry

	for _, geoms := range frameSystemGeometries {
		for _, geom := range geoms.Geometries() {
			// Check if this geometry belongs to checkFrame or its children
			if isPartOfFrame(geom.Label(), checkFrame.Name()) {
				movingGeometries = append(movingGeometries, geom)
			} else {
				staticGeometries = append(staticGeometries, geom)
			}
		}
	}

	// Get world obstacles
	obstaclesInFrame, err := worldstate.ObstaclesInWorldFrame(fs, seedMap.ToFrameSystemInputs())
	if err != nil {
		return err
	}
	worldGeometries := obstaclesInFrame.Geometries()

	// Create collision constraints
	// Note: We pass nil for allowed collisions. Collisions at the start configuration
	// are handled by skipping the first interpolated point in each segment check.
	collisionConstraints, err := motionplan.CreateAllCollisionConstraints(
		fs,
		movingGeometries,
		staticGeometries,
		worldGeometries,
		nil, // no explicitly allowed collisions
		1e-8, // default collision buffer
	)
	if err != nil {
		return err
	}

	// Create a constraint checker
	checker := motionplan.NewEmptyConstraintChecker(nil)
	checker.SetCollisionConstraints(collisionConstraints)

	// Resolution for interpolation (30mm as used in the deleted code)
	const resolution = 30.0

	// Collect all errors
	var collisionErrors []error

	// Check each segment in the trajectory
	for i := 0; i < len(linearTrajectory)-1; i++ {
		segment := &motionplan.SegmentFS{
			StartConfiguration: linearTrajectory[i],
			EndConfiguration:   linearTrajectory[i+1],
			FS:                 fs,
		}

		// Interpolate and check the segment
		interpolatedConfigurations, err := motionplan.InterpolateSegmentFS(segment, resolution)
		if err != nil {
			return err
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

			_, err := checker.CheckStateFSConstraints(ctx, state)
			if err != nil {
				collisionErrors = append(collisionErrors,
					fmt.Errorf("collision in segment %d at interpolation step %d (between waypoint %d and %d): %w",
						i, j, i, i+1, err))
			}
		}
	}

	// Return all errors combined
	if len(collisionErrors) > 0 {
		return fmt.Errorf("found %d collision(s): %v", len(collisionErrors), collisionErrors)
	}

	return nil
}

// isPartOfFrame checks if a geometry label belongs to a specific frame.
// This is a simple string prefix check - geometries are typically labeled as "frameName:geometryName"
func isPartOfFrame(geometryLabel, frameName string) bool {
	// Check if the label starts with the frame name followed by a colon
	prefix := frameName + ":"
	return len(geometryLabel) >= len(prefix) && geometryLabel[:len(prefix)] == prefix
}
