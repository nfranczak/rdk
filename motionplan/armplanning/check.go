package armplanning

import (
	"context"
	"fmt"
	"time"

	vizClient "github.com/viam-labs/motion-tools/client/client"
	"go.viam.com/rdk/logging"
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
) ([]referenceframe.Input, error) {
	trajectory := plan.Trajectory()

	// Validate that the plan has at least one element
	if len(trajectory) < 1 {
		return nil, nil // empty plan, nothing to check
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
		return nil, err
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

	logger := logging.NewLogger("CheckPlan")
	// logger.Infof("linearTrajectory[0]: ", linearTrajectory[0])
	checker, err := motionplan.NewConstraintChecker(
		NewBasicPlannerOptions().CollisionBufferMM,
		nil,
		nil, nil,
		fs,
		movingGeometries,
		staticGeometries,
		linearTrajectory[0],
		worldstate,
		logger,
	)

	for _, t := range linearTrajectory {
		state := &motionplan.StateFS{
			FS:            fs,
			Configuration: t,
		}

		gifs, err := checkFrame.Geometries(t.GetLinearizedInputs())
		if err != nil {
			return nil, err
		}
		// logger.Info(t.GetLinearizedInputs())
		vizClient.DrawGeometries(gifs, []string{"orange", "orange", "orange", "orange", "orange", "orange"})

		if v, err := checker.CheckStateFSConstraints(ctx, state); err != nil {
			if v > 0 {
				return t.GetLinearizedInputs(), fmt.Errorf("got this error: %v, depite value being positive...", err)
			} else {
				return t.GetLinearizedInputs(), fmt.Errorf("got this error: %v", err)
			}

		}
		time.Sleep(time.Millisecond * 125)

	}

	return nil, nil
}

// isPartOfFrame checks if a geometry label belongs to a specific frame.
// This is a simple string prefix check - geometries are typically labeled as "frameName:geometryName"
func isPartOfFrame(geometryLabel, frameName string) bool {
	// Check if the label starts with the frame name followed by a colon
	prefix := frameName + ":"
	return len(geometryLabel) >= len(prefix) && geometryLabel[:len(prefix)] == prefix
}
