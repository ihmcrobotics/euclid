package us.ihmc.euclid.referenceFrame.tools;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Box3D;
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
   public void testBoundingBox3D()
   {
      Random random = new Random(5768787);

      for (int i = 0; i < ITERATIONS; i++)
      { // Box3D: shapeFrame = world, boundingBoxFrame = world
         RigidBodyTransform frameTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         if (random.nextDouble() < 0.3)
            frameTransform.getRotation().setToZero();
         if (random.nextDouble() < 0.3)
            frameTransform.getTranslation().setToZero();
         Box3D boxInFrame = EuclidShapeRandomTools.nextBox3D(random);
         if (random.nextDouble() < 0.3)
            boxInFrame.getOrientation().setToZero();
         if (random.nextDouble() < 0.3)
            boxInFrame.getPosition().setToZero();
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
         RigidBodyTransform frameTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         if (random.nextDouble() < 0.3)
            frameTransform.getRotation().setToZero();
         if (random.nextDouble() < 0.3)
            frameTransform.getTranslation().setToZero();
         ReferenceFrame shapeFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("shapeFrame", worldFrame, frameTransform);
         Box3D boxInFrame = EuclidShapeRandomTools.nextBox3D(random);
         if (random.nextDouble() < 0.3)
            boxInFrame.getOrientation().setToZero();
         if (random.nextDouble() < 0.3)
            boxInFrame.getPosition().setToZero();
         Box3D boxInWorld = new Box3D(boxInFrame);
         shapeFrame.transformFromThisToDesiredFrame(worldFrame, boxInWorld);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         EuclidFrameShapeTools.boundingBoxBox3D(worldFrame, boxInWorld, worldFrame, expected);
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
         Box3D boxInWorld = new Box3D(boxInFrame);
         shapeFrame.transformFromThisToDesiredFrame(boundingBoxFrame, boxInWorld);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         EuclidFrameShapeTools.boundingBoxBox3D(boundingBoxFrame, boxInWorld, boundingBoxFrame, expected);
         EuclidFrameShapeTools.boundingBoxBox3D(shapeFrame, boxInFrame, boundingBoxFrame, actual);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);

         boxInWorld.getBoundingBox(expected);
         EuclidGeometryTestTools.assertBoundingBox3DEquals("Iteration " + i, expected, actual, EPSILON);
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
