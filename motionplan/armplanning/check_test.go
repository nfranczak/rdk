package armplanning

import (
	"context"
	"fmt"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/test"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	frame "go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
)

func TestCheckPlan(t *testing.T) {
	logger := logging.NewTestLogger(t)
	ctx := context.Background()

	// Load UR20e kinematics
	ur20, err := frame.ParseModelJSONFile(utils.ResolveFile("components/arm/fake/kinematics/ur20.json"), "")
	test.That(t, err, test.ShouldBeNil)

	// Create frame system and add the arm
	fs := frame.NewEmptyFrameSystem("test")
	err = fs.AddFrame(ur20, fs.World())
	test.That(t, err, test.ShouldBeNil)

	// Starting joint configuration in radians
	startInputs := []frame.Input{
		0.7853981633974483,
		-0.7853981633974483,
		1.5707963267948966,
		-0.7853981633974483,
		0.7853981633974483,
		0,
	}

	// Create the three wall obstacles
	// Big wall behind the arm
	bigWall, err := spatialmath.NewBox(
		spatialmath.NewPose(
			r3.Vector{X: 499.80892449234604, Y: 0, Z: 0},
			&spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 0},
		),
		r3.Vector{X: 100, Y: 6774.340100002068, Z: 4708.262746117678},
		"bigWall",
	)
	test.That(t, err, test.ShouldBeNil)

	// First little wall
	littleWall1, err := spatialmath.NewBox(
		spatialmath.NewPose(
			r3.Vector{X: -489.0617925579456, Y: 0, Z: 0},
			&spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 0},
		),
		r3.Vector{X: 693.3530661392058, Y: 100, Z: 725.3808831665151},
		"littleWall1",
	)
	test.That(t, err, test.ShouldBeNil)

	// Second little wall
	littleWall2, err := spatialmath.NewBox(
		spatialmath.NewPose(
			r3.Vector{X: -812.5564475789858, Y: 0, Z: 295.11694017940315},
			&spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 0},
		),
		r3.Vector{X: 369.641316443868, Y: 100, Z: 596.3239775519398},
		"littleWall2",
	)
	test.That(t, err, test.ShouldBeNil)

	// Create world state with all obstacles
	worldState, err := frame.NewWorldState(
		[]*frame.GeometriesInFrame{
			frame.NewGeometriesInFrame(frame.World, []spatialmath.Geometry{bigWall, littleWall1, littleWall2}),
		},
		nil,
	)
	test.That(t, err, test.ShouldBeNil)

	// Define the goal pose
	goalPose := spatialmath.NewPose(
		r3.Vector{
			X: -1091.7784630090632,
			Y: 653.2215369909372,
			Z: 171.2573338461849,
		},
		&spatialmath.OrientationVectorDegrees{
			OX:    -0.9999999999999999,
			OY:    -5.551115123125783e-17,
			OZ:    -8.495620873461007e-11,
			Theta: 89.9999999833808,
		},
	)
	
	// Create the plan request
	planRequest := &PlanRequest{
		FrameSystem: fs,
		Goals: []*PlanState{
			{poses: frame.FrameSystemPoses{ur20.Name(): frame.NewPoseInFrame(frame.World, goalPose)}},
		},
		StartState:     &PlanState{structuredConfiguration: frame.FrameSystemInputs{ur20.Name(): startInputs}},
		WorldState:     worldState,
		PlannerOptions: NewBasicPlannerOptions(),
	}

	// Plan the motion
	plan, _, err := PlanMotion(ctx, logger, planRequest)
	fmt.Println(plan.Trajectory())
	test.That(t, err, test.ShouldBeNil)
	test.That(t, plan, test.ShouldNotBeNil)

	// Check the plan for collisions using the original CheckPlan
	_, err = CheckPlan(ctx, worldState, fs, plan)
	test.That(t, err, test.ShouldBeNil)

	// Also test CheckPlanFromRequest using the original plan request
	_, err = CheckPlanFromRequest(ctx, planRequest, plan)
	test.That(t, err, test.ShouldBeNil)
}

// TestCheckPlanWithAllowedCollisions tests that CheckPlan correctly allows collisions that exist at the start configuration.
func TestCheckPlanWithAllowedCollisions(t *testing.T) {
	ctx := context.Background()

	// Load UR5e kinematics
	ur5, err := frame.ParseModelJSONFile(utils.ResolveFile("components/arm/fake/kinematics/ur5e.json"), "")
	test.That(t, err, test.ShouldBeNil)

	// Create frame system and add the arm
	fs := frame.NewEmptyFrameSystem("test")
	err = fs.AddFrame(ur5, fs.World())
	test.That(t, err, test.ShouldBeNil)

	// Starting configuration - use a non-home position
	startInputs := []frame.Input{0, -1.5708, 1.5708, 0, 0, 0}

	// Get geometries at start configuration to find where the forearm link is
	startLinearInputs := frame.FrameSystemInputs{ur5.Name(): startInputs}.ToLinearInputs()
	fsGeoms, err := frame.FrameSystemGeometriesLinearInputs(fs, startLinearInputs)
	test.That(t, err, test.ShouldBeNil)

	// Find the forearm link geometry position
	var forearmCenter r3.Vector
	for _, geomsInFrame := range fsGeoms {
		for _, geom := range geomsInFrame.Geometries() {
			if geom.Label() == "UR5e:forearm_link" {
				forearmCenter = geom.Pose().Point()
				break
			}
		}
	}

	// Create an obstacle that overlaps with the forearm link
	obstacle, err := spatialmath.NewBox(
		spatialmath.NewPose(
			forearmCenter,
			&spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 0},
		),
		r3.Vector{X: 300, Y: 300, Z: 300},
		"obstacle",
	)
	test.That(t, err, test.ShouldBeNil)

	// Create world state with the obstacle
	worldState, err := frame.NewWorldState(
		[]*frame.GeometriesInFrame{
			frame.NewGeometriesInFrame(frame.World, []spatialmath.Geometry{obstacle}),
		},
		nil,
	)
	test.That(t, err, test.ShouldBeNil)

	// Test 1: Plan where robot doesn't move - collision at start should be allowed throughout
	// Create a trajectory with the same configuration repeated (no movement)
	trajectoryNoMove := motionplan.Trajectory{
		{ur5.Name(): startInputs},
		{ur5.Name(): startInputs}, // Same as start
	}

	planNoMove := &testPlan{trajectory: trajectoryNoMove}

	// This should pass because the collision exists at index 0 and persists (same geometry pairs)
	_, err = CheckPlan(ctx, worldState, fs, planNoMove)
	test.That(t, err, test.ShouldBeNil)
}

// TestCheckPlanFromRequest tests the convenience wrapper that takes a PlanRequest.
func TestCheckPlanFromRequest(t *testing.T) {
	ctx := context.Background()

	// Load UR5e kinematics
	ur5, err := frame.ParseModelJSONFile(utils.ResolveFile("components/arm/fake/kinematics/ur5e.json"), "")
	test.That(t, err, test.ShouldBeNil)

	// Create frame system and add the arm
	fs := frame.NewEmptyFrameSystem("test")
	err = fs.AddFrame(ur5, fs.World())
	test.That(t, err, test.ShouldBeNil)

	// Create a simple trajectory
	startInputs := []frame.Input{0, 0, 0, 0, 0, 0}
	goalInputs := []frame.Input{0.5, 0, 0, 0, 0, 0}
	trajectory := motionplan.Trajectory{
		{ur5.Name(): startInputs},
		{ur5.Name(): goalInputs},
	}

	// Create a plan
	plan := &testPlan{trajectory: trajectory}

	// Create a PlanRequest
	planRequest := &PlanRequest{
		FrameSystem: fs,
		WorldState:  &frame.WorldState{},
	}

	// Test CheckPlanFromRequest - should succeed with no obstacles
	_, err = CheckPlanFromRequest(ctx, planRequest, plan)
	test.That(t, err, test.ShouldBeNil)

	// Test with nil request - should fail
	_, err = CheckPlanFromRequest(ctx, nil, plan)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "plan request cannot be nil")

	// Test with nil frame system - should fail
	badRequest := &PlanRequest{
		FrameSystem: nil,
		WorldState:  &frame.WorldState{},
	}
	_, err = CheckPlanFromRequest(ctx, badRequest, plan)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "frame system")

	// Test with nil world state - should fail
	badRequest2 := &PlanRequest{
		FrameSystem: fs,
		WorldState:  nil,
	}
	_, err = CheckPlanFromRequest(ctx, badRequest2, plan)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "world state")
}

// testPlan is a simple implementation of motionplan.Plan for testing.
type testPlan struct {
	trajectory motionplan.Trajectory
}

func (p *testPlan) Trajectory() motionplan.Trajectory {
	return p.trajectory
}

func (p *testPlan) Path() motionplan.Path {
	return nil
}
