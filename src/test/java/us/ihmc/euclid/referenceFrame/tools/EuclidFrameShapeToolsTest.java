package us.ihmc.euclid.referenceFrame.tools;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Capsule3D;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
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
   private static RigidBodyTransform nextRigidBodyTransformWithIdentityEdgeCase(Random random, double rotationIdentityPercentage, double positionZeroPercentage)
   {
      RigidBodyTransform next = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      if (random.nextDouble() < rotationIdentityPercentage)
         next.getRotation().setToZero();
      if (random.nextDouble() < positionZeroPercentage)
         next.getTranslation().setToZero();
      return next;
   }

}
