package referenceframe

import (
	"encoding/xml"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/test"

	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
)

func TestParseURDFFile(t *testing.T) {
	// Test a URDF which has prismatic joints
	u, err := ParseModelXMLFile(utils.ResolveFile("referenceframe/testfiles/example_gantry.xml"), "")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(u.DoF()), test.ShouldEqual, 2)

	// Test a URDF will has collision geometries we can evaluate and a DoF of 6
	u, err = ParseModelXMLFile(utils.ResolveFile("referenceframe/testfiles/ur5e.urdf"), "")
	test.That(t, err, test.ShouldBeNil)
	model, ok := u.(*SimpleModel)
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, u.Name(), test.ShouldEqual, "ur5")
	test.That(t, len(u.DoF()), test.ShouldEqual, 6)
	modelGeo, err := model.Geometries(make([]Input, len(model.DoF())))
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(modelGeo.Geometries()), test.ShouldEqual, 5) // notably we only have 5 geometries for this model

	// Test naming of a URDF to something other than the robot's name element
	u, err = ParseModelXMLFile(utils.ResolveFile("referenceframe/testfiles/ur5e.urdf"), "foo")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, u.Name(), test.ShouldEqual, "foo")
}

func TestWorldStateConversion(t *testing.T) {
	foo, err := spatialmath.NewSphere(spatialmath.NewZeroPose(), 10, "foo")
	test.That(t, err, test.ShouldBeNil)
	bar, err := spatialmath.NewBox(spatialmath.NewZeroPose(), r3.Vector{X: 1, Y: 2, Z: 3}, "bar")
	test.That(t, err, test.ShouldBeNil)
	ws, err := NewWorldState(
		[]*GeometriesInFrame{NewGeometriesInFrame(World, []spatialmath.Geometry{foo, bar})},
		nil,
	)
	test.That(t, err, test.ShouldBeNil)

	cfg, err := NewModelFromWorldState(ws, "test")
	test.That(t, err, test.ShouldBeNil)
	bytes, err := xml.MarshalIndent(cfg, "", "  ")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, bytes, test.ShouldNotBeNil)
}

func TestURDFWithMeshes(t *testing.T) {
	// Test UFactory 850 with meshes
	uf850, err := ParseModelXMLFile(utils.ResolveFile("referenceframe/testfiles/uf850.urdf"), "uf850")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, uf850.Name(), test.ShouldEqual, "uf850")
	test.That(t, len(uf850.DoF()), test.ShouldEqual, 6)

	// Test that the model can compute transforms
	pose, err := uf850.Transform([]Input{0, 0, 0, 0, 0, 0})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, pose, test.ShouldNotBeNil)

	// Verify the model has geometries with meshes
	model, ok := uf850.(*SimpleModel)
	test.That(t, ok, test.ShouldBeTrue)
	geometries, err := model.Geometries(make([]Input, len(model.DoF())))
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(geometries.Geometries()), test.ShouldBeGreaterThan, 0)

	// Verify at least one geometry is a mesh
	hasMesh := false
	for _, geom := range geometries.Geometries() {
		if _, isMesh := geom.(*spatialmath.Mesh); isMesh {
			hasMesh = true
			break
		}
	}
	test.That(t, hasMesh, test.ShouldBeTrue)
}
