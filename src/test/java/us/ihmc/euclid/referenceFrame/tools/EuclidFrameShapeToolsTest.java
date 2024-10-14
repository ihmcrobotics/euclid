package us.ihmc.euclid.referenceFrame.tools;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Capsule3D;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.shape.primitives.Ellipsoid3D;
import us.ihmc.euclid.shape.primitives.Ramp3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;

import static org.junit.jupiter.api.Assertions.*;

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
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);

         boxInWorld.getBoundingBox(expected);
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);
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
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);

         boxInWorld.getBoundingBox(expected);
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);
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
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);

         boxInBBXFrame.getBoundingBox(expected);
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);
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
         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);

         capsuleInWorld.getBoundingBox(expected);
         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
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
         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);

         capsuleInWorld.getBoundingBox(expected);
         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
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
         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);

         capsuleInBBX.getBoundingBox(expected);
         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
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
         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);

         cylinderInWorld.getBoundingBox(expected);
         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
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
         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);

         cylinderInWorld.getBoundingBox(expected);
         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
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
         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);

         cylinderInBBX.getBoundingBox(expected);
         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
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
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);

         ellipsoidInWorld.getBoundingBox(expected);
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);
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
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);

         ellipsoidInWorld.getBoundingBox(expected);
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);
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
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);

         ellipsoidInBBXFrame.getBoundingBox(expected);
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);
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
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);

         rampInWorld.getBoundingBox(expected);
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);
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
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);

         rampInWorld.getBoundingBox(expected);
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);
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
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);

         rampInBBXFrame.getBoundingBox(expected);
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);
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
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);

         sphereInWorld.getBoundingBox(expected);
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);
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
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);

         sphereInWorld.getBoundingBox(expected);
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);
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
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);

         sphereInBBXFrame.getBoundingBox(expected);
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);
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
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);

         convexPolytopeInWorld.getBoundingBox(expected);
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);
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
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);

         convexPolytopeInWorld.getBoundingBox(expected);
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);
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
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);

         convexPolytopeInBBXFrame.getBoundingBox(expected);
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPSILON);
      }
   }

   @Test
   public void testGetDistanceBetweenPointAndPlane1()
   {
      FramePoint3D pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      FrameVector3D planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 0, 1);
      FramePoint3D point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 3);
      double actual = EuclidFrameShapeTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
      double expected = 3.0;
      assertEquals(expected, actual, EPSILON, "FAILED: Distance from point to plane");

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 0, 1);
      point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 3, 3, -3);
      actual = EuclidFrameShapeTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
      expected = 3.0;
      assertEquals(expected, actual, EPSILON, "FAILED: Distance from point to plane");

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 0, 1);
      point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, -3);
      actual = EuclidFrameShapeTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
      expected = 3.0;
      assertEquals(expected, actual, EPSILON, "FAILED: Distance from point to plane");

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 3);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 0, 1);
      point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, -3);
      actual = EuclidFrameShapeTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
      expected = 6.0;
      assertEquals(expected, actual, EPSILON, "FAILED: Distance from point to plane");

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 1, 0, 0);
      point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 3, 0, 0);
      actual = EuclidFrameShapeTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
      expected = 3.0;
      assertEquals(expected, actual, EPSILON, "FAILED: Distance from point to plane");

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 1, 0);
      point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 3, 0);
      actual = EuclidFrameShapeTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
      expected = 3.0;
      assertEquals(expected, actual, EPSILON, "FAILED: Distance from point to plane");

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1, 1, 1);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 1, 0);
      point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 3, 0);
      actual = EuclidFrameShapeTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
      expected = 2.0;
      assertEquals(expected, actual, EPSILON, "FAILED: Distance from point to plane");
   }

   @Test
   public void testIsLineSegmentIntersectingPlane1()
   {
      FramePoint3D pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      FrameVector3D planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 0, 1);
      FramePoint3D lineStart = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, -1);
      FramePoint3D lineEnd = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 3);
      assertTrue(EuclidFrameShapeTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, lineStart, lineEnd));

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 1, 0, 0);
      lineStart = new FramePoint3D(ReferenceFrame.getWorldFrame(), -6, 3, -3);
      lineEnd = new FramePoint3D(ReferenceFrame.getWorldFrame(), 6, 3, 6);
      assertTrue(EuclidFrameShapeTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, lineStart, lineEnd));

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 1, 0);
      lineStart = new FramePoint3D(ReferenceFrame.getWorldFrame(), 6, -3, -3);
      lineEnd = new FramePoint3D(ReferenceFrame.getWorldFrame(), 6, 3, 6);
      assertTrue(EuclidFrameShapeTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, lineStart, lineEnd));

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 0, 1);
      lineStart = new FramePoint3D(ReferenceFrame.getWorldFrame(), 6, -3, 3);
      lineEnd = new FramePoint3D(ReferenceFrame.getWorldFrame(), 6, 3, 6);
      assertFalse(EuclidFrameShapeTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, lineStart, lineEnd));

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 0, 1);
      lineStart = new FramePoint3D(ReferenceFrame.getWorldFrame(), 6, -3, -3);
      lineEnd = new FramePoint3D(ReferenceFrame.getWorldFrame(), 6, 3, -1);
      assertFalse(EuclidFrameShapeTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, lineStart, lineEnd));
   }

   @Test
   public void testGetPerpendicularVectorFromLineToPoint1()
   {
      FramePoint3D point0 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      FramePoint3D lineStart0 = new FramePoint3D(ReferenceFrame.getWorldFrame(), -10, 10, 0);
      FramePoint3D lineEnd0 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 10, 10, 0);
      FramePoint3D intersectionPoint0 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 10, 0);
      FrameVector3D x0 = new FrameVector3D(point0.getReferenceFrame());
      x0.sub(point0, intersectionPoint0);
      FrameVector3D expectedReturn0 = x0;
      FrameVector3D actualReturn0 = EuclidFrameShapeTools.getPerpendicularVectorFromLineToPoint(point0, lineStart0, lineEnd0, intersectionPoint0);

      assertTrue(expectedReturn0.epsilonEquals(actualReturn0, EPSILON), "Test Failed");

      FramePoint3D point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 4, 2, 0);
      FramePoint3D lineStart = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      FramePoint3D lineEnd = new FramePoint3D(ReferenceFrame.getWorldFrame(), 10, 10, 0);
      FramePoint3D intersectionPoint = new FramePoint3D(ReferenceFrame.getWorldFrame(), 3, 3, 0);
      FrameVector3D x = new FrameVector3D(point.getReferenceFrame());
      x.sub(point, intersectionPoint);
      FrameVector3D expectedReturn = x;
      FrameVector3D actualReturn = EuclidFrameShapeTools.getPerpendicularVectorFromLineToPoint(point, lineStart, lineEnd, intersectionPoint);
      assertTrue(expectedReturn.epsilonEquals(actualReturn, EPSILON), "Test Failed");

      FramePoint3D point1 = new FramePoint3D(ReferenceFrame.getWorldFrame(), -2.5, 1.5, 0);
      FramePoint3D lineStart1 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      FramePoint3D lineEnd1 = new FramePoint3D(ReferenceFrame.getWorldFrame(), -4, 4, 0);
      FramePoint3D intersectionPoint1 = new FramePoint3D(ReferenceFrame.getWorldFrame(), -2, 2, 0);

      EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(new Point2D(-2.5, 1.5), new Point2D(0, 0), new Point2D(-4, 4));
      FrameVector3D x1 = new FrameVector3D(point1.getReferenceFrame());
      x1.sub(point1, intersectionPoint1);
      FrameVector3D expectedReturn1 = x1;
      FrameVector3D actualReturn1 = EuclidFrameShapeTools.getPerpendicularVectorFromLineToPoint(point1, lineStart1, lineEnd1, intersectionPoint1);

      assertTrue(expectedReturn1.epsilonEquals(actualReturn1, EPSILON), "Test Failed");
   }

   @Test
   public void testGetPlaneNormalGivenThreePoints()
   {
      FramePoint3D point1 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 5, 0);
      FramePoint3D point2 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 5, 0);
      FramePoint3D point3 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 5, 0);
      FrameVector3D expectedReturn = null;
      FrameVector3D actualReturn = EuclidFrameShapeTools.getPlaneNormalGivenThreePoints(point1, point2, point3);
      assertEquals(expectedReturn, actualReturn, "test failed");

      FramePoint3D point91 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 3, 0);
      FramePoint3D point92 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 5, 0);
      FramePoint3D point93 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 1, 0);
      FrameVector3D expectedReturn9 = null;
      FrameVector3D actualReturn9 = EuclidFrameShapeTools.getPlaneNormalGivenThreePoints(point91, point92, point93);
      assertEquals(expectedReturn9, actualReturn9, "test failed");

      FramePoint3D point81 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 9, 0, 0);
      FramePoint3D point82 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 7, 0, 0);
      FramePoint3D point83 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 4, 0, 0);
      FrameVector3D expectedReturn8 = null;
      FrameVector3D actualReturn8 = EuclidFrameShapeTools.getPlaneNormalGivenThreePoints(point81, point82, point83);
      assertEquals(expectedReturn8, actualReturn8, "test failed");

      FramePoint3D point71 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 4);
      FramePoint3D point72 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 6);
      FramePoint3D point73 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 7);
      FrameVector3D expectedReturn7 = null;
      FrameVector3D actualReturn7 = EuclidFrameShapeTools.getPlaneNormalGivenThreePoints(point71, point72, point73);
      assertEquals(expectedReturn7, actualReturn7, "test failed");

      FramePoint3D point11 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 5, 46);
      FramePoint3D point12 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 587, 3);
      FramePoint3D point13 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 18, 8);
      FramePoint3D p1 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 5, 5);
      FramePoint3D v1 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1, 5, 5);
      FrameVector3D expectedReturn1 = new FrameVector3D(p1.getReferenceFrame());
      expectedReturn1.sub(p1, v1);
      FrameVector3D actualReturn1 = EuclidFrameShapeTools.getPlaneNormalGivenThreePoints(point11, point12, point13);
      assertTrue(expectedReturn1.epsilonEquals(actualReturn1, EPSILON), "Test Failed");

      FramePoint3D point21 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 65, 0, 46);
      FramePoint3D point22 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 43, 0, 3);
      FramePoint3D point23 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 13, 0, 8);
      FramePoint3D p2 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 1, 5);
      FramePoint3D v2 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 5);
      FrameVector3D expectedReturn2 = new FrameVector3D(p2.getReferenceFrame());
      expectedReturn2.sub(p2, v2);
      FrameVector3D actualReturn2 = EuclidFrameShapeTools.getPlaneNormalGivenThreePoints(point21, point22, point23);
      assertTrue(expectedReturn2.epsilonEquals(actualReturn2, EPSILON), "Test Failed");

      FramePoint3D point31 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 65, 56, 0);
      FramePoint3D point32 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 43, 3, 0);
      FramePoint3D point33 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 13, 87, 0);
      FramePoint3D p3 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 55, 0);
      FramePoint3D v3 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 55, 1);
      FrameVector3D expectedReturn3 = new FrameVector3D(p3.getReferenceFrame());
      expectedReturn3.sub(p3, v3);
      FrameVector3D actualReturn3 = EuclidFrameShapeTools.getPlaneNormalGivenThreePoints(point31, point32, point33);
      assertTrue(expectedReturn3.epsilonEquals(actualReturn3, EPSILON), "Test Failed");
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
