# ![Euclid](logo/Euclid.png)

![euclid](https://maven-badges.herokuapp.com/maven-central/us.ihmc/euclid/badge.svg?style=plastic)
![buildstatus](https://github.com/ihmcrobotics/euclid/actions/workflows/gradle.yml/badge.svg)

BONSOIR

## Minutiae

### Licensing
This library is under the license Apache 2.0. Consult the license file for more information.

### Compatibility
This library is compatible with Java 8+.

### Dependency
This library sources depends on the matrix library EJML [here](http://ejml.org/), while the tests also on JUnit 5 and PIT mutation testing library [here](http://pitest.org/).

## What is Euclid?
Euclid is a general library addressing vector math and geometry problems.
The library is born from the need of having a base library for geometric applications that is well tested, flexible, and meant for real-time environment where
garbage generation is not allowed.
Euclid is composed of 5 modules.
- Euclid (previously Euclid Core): Defines the base vector math such as points, vectors, orientations, and transforms.
	It also puts in place a general framework that heavily relies on interfaces which helps: identifying objects that are not meant to be modified, e.g. `Tuple3DReadOnly`, and simplifying the use of custom implementations by either implementing the read-only or basics interface. 
- Euclid Geometry: Defines simple geometries such as lines, line-segments, bounding boxes, planes, and convex polygons.
	It comes with a large variety of tools which can either be found in the geometry implementations such as `ConvexPolygon2D` or in `EuclidGeometryTools` and `EuclidGeometryPolygonTools`.
- Euclid Frame: Framework introducing the notion of reference frames and re-defines the types in Euclid Core & Geometry to attach them to reference frames and add options to easily transform object from a frame to another.
	This module comes from the frustration of never knowing the right combination of transforms to get a set of coordinates that currently are expressed with respect to a referential to be expressed with respect to another object or referential.
	It is very similar to the ROS tf framework.
- Euclid Shape: Defines 3D shapes such as boxes, cylinders, spheres, ellipsoids, and convex polytopes. It also provides collision detection algorithms. Additional shapes, ray casting, and other geometric operations will be added in future releases.
- Euclid Frame Shape (under development): Expansion of the 3D shape framework to include the reference frame concept.

This library is continuously being improved and will be extended in the near future as the result of migrating progressively content from the IHMC open source software [here](https://github.com/ihmcrobotics/ihmc-open-robotics-software).

## Who would use Euclid?
Any software developer manipulating geometry objects or dealing with 2D or 3D graphical UI is susceptible to use this library as the base of a more complex algorithm.

## What is the goal of Euclid?
The goal for Euclid is to become the most flexible, easy to use, and fast library for geometry applications, so it results in great increase in development productivity.

## How can I contribute to Euclid?
Please read [CONTRIBUTING.md](https://github.com/ihmcrobotics/euclid/blob/develop/CONTRIBUTING.md).

## Content
Note that most of the modules define the following tools:
- `Euclid(...)RandomTools` defines static methods to the objects defined in the module randomly.
- `Euclid(...)TestTools` defines assertions for the objects defined in the modules.
- `Euclid(...)IOTools` defines static methods to create a representative `String` of any object defined in the module.
- `Euclid(...)Tools` defines static methods that gather the actual computation used in the different objects defined in the module.

For more information about the static tools, see [Wiki- Static tools summary](https://github.com/ihmcrobotics/euclid/wiki/Static-tools-summary).

#### Euclid
- Points in 2 and 3 dimensions: `Point2D`, `Point3D`, and their single-float precision defined counterparts: `Point2D32`, `Point3D32`.
- Vectors in 2, 3, and 4 dimensions: `Vector2D`, `Vector3D`, `Vector4D`, and their single-float precision defined counterparts: `Vector2D32`, `Vector3D32`, `Vector4D32`.
- Unit vectors in 2 and 3 dimensions: `UnitVector2D` and `UnitVector3D`.
- The definition of several representations of 3D orientations:
	- as a 3-by-3 rotation matrix: `RotationMatrix`.
	- as a quaternion: `Quaternion`, `Quaternion32`.
	- as an axis-angle: `AxisAngle`, `AxisAngle32`.
	- as yaw-pitch-roll angles: `YawPitchRoll`.
	- it also provides tools to convert any rotation definition to a rotation vector, also called Euler vector.
- A set of tools to easily convert from one orientation definition to another: `AxisAngleConversion`, `QuaternionConversion`, `RotationMatrixConversion`, `RotationVectorConversion`, and `YawPitchRollConversion`.
- A general 3-by-3 matrix: `Matrix3D`.
- A 3-by-3 matrix for rotating and scaling geometry objects: `RotationScaleMatrix`.
- Geometric transforms:
	 - A 4-by-4 homogeneous matrix for rotating and translating geometry objects: `RigidBodyTransform`.
	 - The more concise equivalent of the RigidBodyTransform using a quaternion instead of a 3-by-3 rotation matrix: `QuaternionBasedTransform`.
	 - A 4-by-4 homogeneous matrix for scaling, rotation, and translating geometry objects: `AffineTransform`.

#### Euclid Geometry
- Infinitely long lines in 2 and 3 dimensions: `Line2D`, `Line3D`.
- Line segments in 2 and 3 dimensions: `LineSegment2D`, `LineSegment3D`.
- Axis-aligned bounding boxes in 2 and 3 dimensions: `BoundingBox2D`, `BoundingBox3D`.
- Convex polygons in 2 dimensions: `ConvexPolygon2D`.
- Infinitely long planes in 3 dimensions: `Plane3D`.
- Poses in 2 and 3 dimensions: `Pose2D` and `Pose3D`.

#### Euclid Frame
- Reference frames in 3 dimensions: `ReferenceFrame`. 
- Contains the equivalent to most of the previous types where the geometry is represented in the same manner but simply with specification of the reference frame in which it is expressed. For instance, the equivalent of `Point3D` is `FramePoint3D`.
- Frame geometries are expansions based on frameless geometries and which interfaces greatly overlap. Using this property, a reflection-based testing framework is provided to test basic features of a frame geometry, such as checking reference frames of two frame geometries match, testing that its API is complete with respect to its frameless counterpart, and testing that the implementation remains consistent with its frameless counterpart. The main entry point for the testing framework is `EuclidFrameAPITester`.

#### Euclid Shape
- Boxes in 3 dimensions: `Box3D`.
- Capsules in 3 dimensions: `Capsule3D`.
- Cylinders in 3 dimensions: `Cylinder3D`.
- Spheres and ellipsoids in 3 dimensions: `Sphere3D` and `Ellipsoid3D`.
- Ramps/Wedges in 3 dimensions: `Ramp3D`.
- Tori in 3 dimensions: `Torus3D`.
- Convex polytopes in 3 dimensions: `ConvexPolytope3D`.
- Collision detection between primitive shapes: `EuclidShapeCollisionTools`.
- Gilbert-Johnson-Keerthi algorithm for collision detection: `GilbertJohnsonKeerthiCollisionDetector`.
- Expanding polytope algorithm for collision detection: `ExpandingPolytopeAlgorithm`.

### Euclid Frame Shape
This module expands the shapes defined in the shape module to include the reference frame framework. The Gilbert-Johnson-Keerthi and Expanding polytope algorithms are also expanded to handle shapes expressed in different reference frames.

## Using Euclid from .jar releases with Maven/Gradle
The releases .jars for Euclid are hosted on Maven repository.
You can browse the IHMC release packages at https://mvnrepository.com/artifact/us.ihmc.

At a minimum, you will need to have the following repository declared in your build script to use the Euclid .jars:

```gradle
repositories {
   mavenCentral()
}
```

Euclid currently publishes 5 distinct modules of interest. Here is an example for adding the dependency to each module of Euclid using your build script:

```gradle
dependencies {
   compile group: "us.ihmc", name: "euclid", version: "x.x"
   compile group: "us.ihmc", name: "euclid-geometry", version: "x.x"
   compile group: "us.ihmc", name: "euclid-frame", version: "x.x"
   compile group: "us.ihmc", name: "euclid-shape", version: "x.x"
   compile group: "us.ihmc", name: "euclid-frame-shape", version: "x.x"
}
```
