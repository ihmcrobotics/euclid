package us.ihmc.euclid.referenceFrame.tools;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.primitives.*;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class EuclidFrameShapeToolsTest
{
   private static final int ITERATIONS = 5000;
   private static final double EPSILON = 1.0e-12;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Test
   public void testBoundingBoxBox3D()
   {
      Random random = new Random(5768787);

      for (int i = 0; i < ITERATIONS; i++)
      { // Box3D: shapeFrame = world, boundingBoxFrame = world
         Box3D boxInFrame = EuclidShapeRandomTools.nextBox3D(random);
         boxInFrame.getPose().set(nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3));
         Box3D boxInWorld = new Box3D(boxInFrame);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         EuclidShapeTools.boundingBoxBox3D(boxInWorld.getPosition(), boxInWorld.getOrientation(), boxInWorld.getSize(), expected);
         EuclidFrameShapeTools.boundingBoxBox3D(worldFrame, boxInFrame, worldFrame, actual);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);

         boxInWorld.getBoundingBox(expected);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Box3D: shapeFrame != world, boundingBoxFrame = world
         RigidBodyTransform shapeFrameTransform = nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3);
         ReferenceFrame shapeFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("shapeFrame", worldFrame, shapeFrameTransform);
         Box3D boxInFrame = EuclidShapeRandomTools.nextBox3D(random);
         boxInFrame.getPose().set(nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3));
         Box3D boxInWorld = new Box3D(boxInFrame);
         shapeFrame.transformFromThisToDesiredFrame(worldFrame, boxInWorld);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         EuclidShapeTools.boundingBoxBox3D(boxInWorld.getPosition(), boxInWorld.getOrientation(), boxInWorld.getSize(), expected);
         EuclidFrameShapeTools.boundingBoxBox3D(shapeFrame, boxInFrame, worldFrame, actual);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);

         boxInWorld.getBoundingBox(expected);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Box3D: shapeFrame != world, boundingBoxFrame != world
         RigidBodyTransform shapeFrameTransform = nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3);
         RigidBodyTransform boundingBoxFrameTransform = nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3);

         ReferenceFrame shapeFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("shapeFrame", worldFrame, shapeFrameTransform);
         ReferenceFrame boundingBoxFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("boundingBoxFrame",
                                                                                                             worldFrame,
                                                                                                             boundingBoxFrameTransform);
         Box3D boxInFrame = EuclidShapeRandomTools.nextBox3D(random);
         boxInFrame.getPose().set(nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3));
         Box3D boxInBBXFrame = new Box3D(boxInFrame);
         shapeFrame.transformFromThisToDesiredFrame(boundingBoxFrame, boxInBBXFrame);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         EuclidShapeTools.boundingBoxBox3D(boxInBBXFrame.getPosition(), boxInBBXFrame.getOrientation(), boxInBBXFrame.getSize(), expected);
         EuclidFrameShapeTools.boundingBoxBox3D(shapeFrame, boxInFrame, boundingBoxFrame, actual);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);

         boxInBBXFrame.getBoundingBox(expected);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);
      }
   }

   @Test
   public void testBoundingBoxCapsule3D()
   {
      Random random = new Random(5768787);

      for (int i = 0; i < ITERATIONS; i++)
      { // Capsule3D: shapeFrame = world, boundingBoxFrame = world
         Capsule3D capsuleInFrame = EuclidShapeRandomTools.nextCapsule3D(random);
         Capsule3D capsuleInWorld = new Capsule3D(capsuleInFrame);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         capsuleInWorld.getBoundingBox(expected);
         EuclidFrameShapeTools.boundingBoxCapsule3D(worldFrame, capsuleInFrame, worldFrame, actual);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expected, actual, EPSILON);

         capsuleInWorld.getBoundingBox(expected);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Capsule3D: shapeFrame != world, boundingBoxFrame = world
         RigidBodyTransform shapeFrameTransform = nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3);
         ReferenceFrame shapeFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("shapeFrame", worldFrame, shapeFrameTransform);
         Capsule3D capsuleInFrame = EuclidShapeRandomTools.nextCapsule3D(random);
         Capsule3D capsuleInWorld = new Capsule3D(capsuleInFrame);
         shapeFrame.transformFromThisToDesiredFrame(worldFrame, capsuleInWorld);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         capsuleInWorld.getBoundingBox(expected);
         EuclidFrameShapeTools.boundingBoxCapsule3D(shapeFrame, capsuleInFrame, worldFrame, actual);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expected, actual, EPSILON);

         capsuleInWorld.getBoundingBox(expected);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Capsule3D: shapeFrame != world, boundingBoxFrame != world
         RigidBodyTransform shapeFrameTransform = nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3);
         RigidBodyTransform boundingBoxFrameTransform = nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3);

         ReferenceFrame shapeFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("shapeFrame", worldFrame, shapeFrameTransform);
         ReferenceFrame boundingBoxFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("boundingBoxFrame",
                                                                                                             worldFrame,
                                                                                                             boundingBoxFrameTransform);
         Capsule3D capsuleInFrame = EuclidShapeRandomTools.nextCapsule3D(random);
         Capsule3D capsuleInBBX = new Capsule3D(capsuleInFrame);
         shapeFrame.transformFromThisToDesiredFrame(boundingBoxFrame, capsuleInBBX);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         capsuleInBBX.getBoundingBox(expected);
         EuclidFrameShapeTools.boundingBoxCapsule3D(shapeFrame, capsuleInFrame, boundingBoxFrame, actual);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expected, actual, EPSILON);

         capsuleInBBX.getBoundingBox(expected);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testBoundingBoxCylinder3D()
   {
      Random random = new Random(5768787);

      for (int i = 0; i < ITERATIONS; i++)
      { // Cylinder3D: shapeFrame = world, boundingBoxFrame = world
         Cylinder3D cylinderInFrame = EuclidShapeRandomTools.nextCylinder3D(random);
         Cylinder3D cylinderInWorld = new Cylinder3D(cylinderInFrame);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         cylinderInWorld.getBoundingBox(expected);
         EuclidFrameShapeTools.boundingBoxCylinder3D(worldFrame, cylinderInFrame, worldFrame, actual);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expected, actual, EPSILON);

         cylinderInWorld.getBoundingBox(expected);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Cylinder3D: shapeFrame != world, boundingBoxFrame = world
         RigidBodyTransform shapeFrameTransform = nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3);
         ReferenceFrame shapeFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("shapeFrame", worldFrame, shapeFrameTransform);
         Cylinder3D cylinderInFrame = EuclidShapeRandomTools.nextCylinder3D(random);
         Cylinder3D cylinderInWorld = new Cylinder3D(cylinderInFrame);
         shapeFrame.transformFromThisToDesiredFrame(worldFrame, cylinderInWorld);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         cylinderInWorld.getBoundingBox(expected);
         EuclidFrameShapeTools.boundingBoxCylinder3D(shapeFrame, cylinderInFrame, worldFrame, actual);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expected, actual, EPSILON);

         cylinderInWorld.getBoundingBox(expected);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Cylinder3D: shapeFrame != world, boundingBoxFrame != world
         RigidBodyTransform shapeFrameTransform = nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3);
         RigidBodyTransform boundingBoxFrameTransform = nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3);

         ReferenceFrame shapeFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("shapeFrame", worldFrame, shapeFrameTransform);
         ReferenceFrame boundingBoxFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("boundingBoxFrame",
                                                                                                             worldFrame,
                                                                                                             boundingBoxFrameTransform);
         Cylinder3D cylinderInFrame = EuclidShapeRandomTools.nextCylinder3D(random);
         Cylinder3D cylinderInBBX = new Cylinder3D(cylinderInFrame);
         shapeFrame.transformFromThisToDesiredFrame(boundingBoxFrame, cylinderInBBX);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         cylinderInBBX.getBoundingBox(expected);
         EuclidFrameShapeTools.boundingBoxCylinder3D(shapeFrame, cylinderInFrame, boundingBoxFrame, actual);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expected, actual, EPSILON);

         cylinderInBBX.getBoundingBox(expected);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testBoundingBoxEllipsoid3D()
   {
      Random random = new Random(5768787);

      for (int i = 0; i < ITERATIONS; i++)
      { // Ellipsoid3D: shapeFrame = world, boundingBoxFrame = world
         Ellipsoid3D ellipsoidInFrame = EuclidShapeRandomTools.nextEllipsoid3D(random);
         ellipsoidInFrame.getPose().set(nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3));
         Ellipsoid3D ellipsoidInWorld = new Ellipsoid3D(ellipsoidInFrame);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         EuclidShapeTools.boundingBoxEllipsoid3D(ellipsoidInWorld.getPosition(), ellipsoidInWorld.getOrientation(), ellipsoidInWorld.getRadii(), expected);
         EuclidFrameShapeTools.boundingBoxEllipsoid3D(worldFrame, ellipsoidInFrame, worldFrame, actual);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);

         ellipsoidInWorld.getBoundingBox(expected);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Ellipsoid3D: shapeFrame != world, boundingBoxFrame = world
         RigidBodyTransform shapeFrameTransform = nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3);
         ReferenceFrame shapeFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("shapeFrame", worldFrame, shapeFrameTransform);
         Ellipsoid3D ellipsoidInFrame = EuclidShapeRandomTools.nextEllipsoid3D(random);
         ellipsoidInFrame.getPose().set(nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3));
         Ellipsoid3D ellipsoidInWorld = new Ellipsoid3D(ellipsoidInFrame);
         shapeFrame.transformFromThisToDesiredFrame(worldFrame, ellipsoidInWorld);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         EuclidShapeTools.boundingBoxEllipsoid3D(ellipsoidInWorld.getPosition(), ellipsoidInWorld.getOrientation(), ellipsoidInWorld.getRadii(), expected);
         EuclidFrameShapeTools.boundingBoxEllipsoid3D(shapeFrame, ellipsoidInFrame, worldFrame, actual);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);

         ellipsoidInWorld.getBoundingBox(expected);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Ellipsoid3D: shapeFrame != world, boundingBoxFrame != world
         RigidBodyTransform shapeFrameTransform = nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3);
         RigidBodyTransform boundingBoxFrameTransform = nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3);

         ReferenceFrame shapeFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("shapeFrame", worldFrame, shapeFrameTransform);
         ReferenceFrame boundingBoxFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("boundingBoxFrame",
                                                                                                             worldFrame,
                                                                                                             boundingBoxFrameTransform);
         Ellipsoid3D ellipsoidInFrame = EuclidShapeRandomTools.nextEllipsoid3D(random);
         ellipsoidInFrame.getPose().set(nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3));
         Ellipsoid3D ellipsoidInBBXFrame = new Ellipsoid3D(ellipsoidInFrame);
         shapeFrame.transformFromThisToDesiredFrame(boundingBoxFrame, ellipsoidInBBXFrame);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         EuclidShapeTools.boundingBoxEllipsoid3D(ellipsoidInBBXFrame.getPosition(),
                                                 ellipsoidInBBXFrame.getOrientation(),
                                                 ellipsoidInBBXFrame.getRadii(),
                                                 expected);
         EuclidFrameShapeTools.boundingBoxEllipsoid3D(shapeFrame, ellipsoidInFrame, boundingBoxFrame, actual);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);

         ellipsoidInBBXFrame.getBoundingBox(expected);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);
      }
   }

   @Test
   public void testBoundingBoxRamp3D()
   {
      Random random = new Random(5768787);

      for (int i = 0; i < ITERATIONS; i++)
      { // Ramp3D: shapeFrame = world, boundingBoxFrame = world
         Ramp3D rampInFrame = EuclidShapeRandomTools.nextRamp3D(random);
         rampInFrame.getPose().set(nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3));
         Ramp3D rampInWorld = new Ramp3D(rampInFrame);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         rampInWorld.getBoundingBox(expected);
         EuclidFrameShapeTools.boundingBoxRamp3D(worldFrame, rampInFrame, worldFrame, actual);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);

         rampInWorld.getBoundingBox(expected);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Ramp3D: shapeFrame != world, boundingBoxFrame = world
         RigidBodyTransform shapeFrameTransform = nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3);
         ReferenceFrame shapeFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("shapeFrame", worldFrame, shapeFrameTransform);
         Ramp3D rampInFrame = EuclidShapeRandomTools.nextRamp3D(random);
         rampInFrame.getPose().set(nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3));
         Ramp3D rampInWorld = new Ramp3D(rampInFrame);
         shapeFrame.transformFromThisToDesiredFrame(worldFrame, rampInWorld);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         rampInWorld.getBoundingBox(expected);
         EuclidFrameShapeTools.boundingBoxRamp3D(shapeFrame, rampInFrame, worldFrame, actual);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);

         rampInWorld.getBoundingBox(expected);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Ramp3D: shapeFrame != world, boundingBoxFrame != world
         RigidBodyTransform shapeFrameTransform = nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3);
         RigidBodyTransform boundingBoxFrameTransform = nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3);

         ReferenceFrame shapeFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("shapeFrame", worldFrame, shapeFrameTransform);
         ReferenceFrame boundingBoxFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("boundingBoxFrame",
                                                                                                             worldFrame,
                                                                                                             boundingBoxFrameTransform);
         Ramp3D rampInFrame = EuclidShapeRandomTools.nextRamp3D(random);
         rampInFrame.getPose().set(nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3));
         Ramp3D rampInBBXFrame = new Ramp3D(rampInFrame);
         shapeFrame.transformFromThisToDesiredFrame(boundingBoxFrame, rampInBBXFrame);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         rampInBBXFrame.getBoundingBox(expected);
         EuclidFrameShapeTools.boundingBoxRamp3D(shapeFrame, rampInFrame, boundingBoxFrame, actual);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);

         rampInBBXFrame.getBoundingBox(expected);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);
      }
   }

   @Test
   public void testBoundingBoxSphere3D()
   {
      Random random = new Random(5768787);

      for (int i = 0; i < ITERATIONS; i++)
      { // Sphere3D: shapeFrame = world, boundingBoxFrame = world
         Sphere3D sphereInFrame = EuclidShapeRandomTools.nextSphere3D(random);
         Sphere3D sphereInWorld = new Sphere3D(sphereInFrame);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         sphereInWorld.getBoundingBox(expected);
         EuclidFrameShapeTools.boundingBoxSphere3D(worldFrame, sphereInFrame, worldFrame, actual);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);

         sphereInWorld.getBoundingBox(expected);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Sphere3D: shapeFrame != world, boundingBoxFrame = world
         RigidBodyTransform shapeFrameTransform = nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3);
         ReferenceFrame shapeFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("shapeFrame", worldFrame, shapeFrameTransform);
         Sphere3D sphereInFrame = EuclidShapeRandomTools.nextSphere3D(random);
         Sphere3D sphereInWorld = new Sphere3D(sphereInFrame);
         shapeFrame.transformFromThisToDesiredFrame(worldFrame, sphereInWorld);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         sphereInWorld.getBoundingBox(expected);
         EuclidFrameShapeTools.boundingBoxSphere3D(shapeFrame, sphereInFrame, worldFrame, actual);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);

         sphereInWorld.getBoundingBox(expected);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Sphere3D: shapeFrame != world, boundingBoxFrame != world
         RigidBodyTransform shapeFrameTransform = nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3);
         RigidBodyTransform boundingBoxFrameTransform = nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3);

         ReferenceFrame shapeFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("shapeFrame", worldFrame, shapeFrameTransform);
         ReferenceFrame boundingBoxFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("boundingBoxFrame",
                                                                                                             worldFrame,
                                                                                                             boundingBoxFrameTransform);
         Sphere3D sphereInFrame = EuclidShapeRandomTools.nextSphere3D(random);
         Sphere3D sphereInBBXFrame = new Sphere3D(sphereInFrame);
         shapeFrame.transformFromThisToDesiredFrame(boundingBoxFrame, sphereInBBXFrame);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         sphereInBBXFrame.getBoundingBox(expected);
         EuclidFrameShapeTools.boundingBoxSphere3D(shapeFrame, sphereInFrame, boundingBoxFrame, actual);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);

         sphereInBBXFrame.getBoundingBox(expected);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);
      }
   }

   @Test
   public void testBoundingBoxConvexPolytope3D()
   {
      Random random = new Random(5768787);

      for (int i = 0; i < ITERATIONS; i++)
      { // ConvexPolytope3D: shapeFrame = world, boundingBoxFrame = world
         ConvexPolytope3D convexPolytopeInFrame = EuclidShapeRandomTools.nextConvexPolytope3D(random);
         convexPolytopeInFrame.applyTransform(nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3));
         ConvexPolytope3D convexPolytopeInWorld = new ConvexPolytope3D(convexPolytopeInFrame);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         convexPolytopeInWorld.getBoundingBox(expected);
         EuclidFrameShapeTools.boundingBoxConvexPolytope3D(worldFrame, convexPolytopeInFrame, worldFrame, actual);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);

         convexPolytopeInWorld.getBoundingBox(expected);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // ConvexPolytope3D: shapeFrame != world, boundingBoxFrame = world
         RigidBodyTransform shapeFrameTransform = nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3);
         ReferenceFrame shapeFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("shapeFrame", worldFrame, shapeFrameTransform);
         ConvexPolytope3D convexPolytopeInFrame = EuclidShapeRandomTools.nextConvexPolytope3D(random);
         convexPolytopeInFrame.applyTransform(nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3));
         ConvexPolytope3D convexPolytopeInWorld = new ConvexPolytope3D(convexPolytopeInFrame);
         shapeFrame.transformFromThisToDesiredFrame(worldFrame, convexPolytopeInWorld);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         convexPolytopeInWorld.getBoundingBox(expected);
         EuclidFrameShapeTools.boundingBoxConvexPolytope3D(shapeFrame, convexPolytopeInFrame, worldFrame, actual);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);

         convexPolytopeInWorld.getBoundingBox(expected);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // ConvexPolytope3D: shapeFrame != world, boundingBoxFrame != world
         RigidBodyTransform shapeFrameTransform = nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3);
         RigidBodyTransform boundingBoxFrameTransform = nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3);

         ReferenceFrame shapeFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("shapeFrame", worldFrame, shapeFrameTransform);
         ReferenceFrame boundingBoxFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("boundingBoxFrame",
                                                                                                             worldFrame,
                                                                                                             boundingBoxFrameTransform);
         ConvexPolytope3D convexPolytopeInFrame = EuclidShapeRandomTools.nextConvexPolytope3D(random);
         convexPolytopeInFrame.applyTransform(nextRigidBodyTransformWithIdentityEdgeCase(random, 0.3, 0.3));
         ConvexPolytope3D convexPolytopeInBBXFrame = new ConvexPolytope3D(convexPolytopeInFrame);
         shapeFrame.transformFromThisToDesiredFrame(boundingBoxFrame, convexPolytopeInBBXFrame);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         convexPolytopeInBBXFrame.getBoundingBox(expected);
         EuclidFrameShapeTools.boundingBoxConvexPolytope3D(shapeFrame, convexPolytopeInFrame, boundingBoxFrame, actual);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);

         convexPolytopeInBBXFrame.getBoundingBox(expected);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);
      }
   }

   private static RigidBodyTransform nextRigidBodyTransformWithIdentityEdgeCase(Random random, double rotationIdentityPercentage, double positionZeroPercentage)
   {
      RigidBodyTransform next = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      //      if (random.nextDouble() < rotationIdentityPercentage)
      next.getRotation().setToZero();
      if (random.nextDouble() < positionZeroPercentage)
         next.getTranslation().setToZero();
      return next;
   }

}
