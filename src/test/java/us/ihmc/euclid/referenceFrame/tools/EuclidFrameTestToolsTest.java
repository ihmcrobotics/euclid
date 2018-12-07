package us.ihmc.euclid.referenceFrame.tools;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.FrameVector4D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple4DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector4DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTestToolsTest;

public class EuclidFrameTestToolsTest
{
   private static final double EPSILON = 0.0001;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Test
   public void testAssertRotationFrameVectorGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame("frameA", random, worldFrame);
      ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame("frameB", random, worldFrame);

      String methodName = "assertRotationFrameVectorGeometricallyEquals";
      Class<FrameVector3DReadOnly> argumentsClass = FrameVector3DReadOnly.class;

      {
         FrameVector3D expected = null;
         FrameVector3D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector3D expected = EuclidFrameRandomTools.nextFrameVector3D(random, frameA);
         FrameVector3D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector3D expected = null;
         FrameVector3D actual = EuclidFrameRandomTools.nextFrameVector3D(random, frameA);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector3D expected = EuclidFrameRandomTools.nextFrameVector3D(random, frameA);
         FrameVector3D actual = EuclidFrameRandomTools.nextFrameVector3D(random, frameA);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector3D expected = EuclidFrameRandomTools.nextFrameVector3D(random, frameA);
         FrameVector3D actual = new FrameVector3D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector3D expected = EuclidFrameRandomTools.nextFrameVector3D(random, frameA);
         FrameVector3D actual = new FrameVector3D(frameB, expected);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertFrameTuple2DEquals() throws Throwable
   {
      Random random = new Random(453453);
      ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame("frameA", random, worldFrame);
      ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame("frameB", random, worldFrame);

      String methodName = "assertFrameTuple2DEquals";
      Class<FrameTuple2DReadOnly> argumentsClass = FrameTuple2DReadOnly.class;

      {
         FrameVector2D expected = null;
         FrameVector2D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector2D expected = EuclidFrameRandomTools.nextFrameVector2D(random, frameA);
         FrameVector2D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector2D expected = null;
         FrameVector2D actual = EuclidFrameRandomTools.nextFrameVector2D(random, frameA);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector2D expected = EuclidFrameRandomTools.nextFrameVector2D(random, frameA);
         FrameVector2D actual = EuclidFrameRandomTools.nextFrameVector2D(random, frameA);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector2D expected = EuclidFrameRandomTools.nextFrameVector2D(random, frameA);
         FrameVector2D actual = new FrameVector2D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector2D expected = EuclidFrameRandomTools.nextFrameVector2D(random, frameA);
         FrameVector2D actual = new FrameVector2D(frameB, expected);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertFramePoint2DGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame("frameA", random, worldFrame);
      ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame("frameB", random, worldFrame);

      String methodName = "assertFramePoint2DGeometricallyEquals";
      Class<FramePoint2DReadOnly> argumentsClass = FramePoint2DReadOnly.class;

      {
         FramePoint2D expected = null;
         FramePoint2D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FramePoint2D expected = EuclidFrameRandomTools.nextFramePoint2D(random, frameA);
         FramePoint2D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FramePoint2D expected = null;
         FramePoint2D actual = EuclidFrameRandomTools.nextFramePoint2D(random, frameA);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FramePoint2D expected = EuclidFrameRandomTools.nextFramePoint2D(random, frameA);
         FramePoint2D actual = EuclidFrameRandomTools.nextFramePoint2D(random, frameA);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FramePoint2D expected = EuclidFrameRandomTools.nextFramePoint2D(random, frameA);
         FramePoint2D actual = new FramePoint2D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FramePoint2D expected = EuclidFrameRandomTools.nextFramePoint2D(random, frameA);
         FramePoint2D actual = new FramePoint2D(frameB, expected);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertFrameVector2DGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame("frameA", random, worldFrame);
      ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame("frameB", random, worldFrame);

      String methodName = "assertFrameVector2DGeometricallyEquals";
      Class<FrameVector2DReadOnly> argumentsClass = FrameVector2DReadOnly.class;

      {
         FrameVector2D expected = null;
         FrameVector2D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector2D expected = EuclidFrameRandomTools.nextFrameVector2D(random, frameA);
         FrameVector2D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector2D expected = null;
         FrameVector2D actual = EuclidFrameRandomTools.nextFrameVector2D(random, frameA);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector2D expected = EuclidFrameRandomTools.nextFrameVector2D(random, frameA);
         FrameVector2D actual = EuclidFrameRandomTools.nextFrameVector2D(random, frameA);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector2D expected = EuclidFrameRandomTools.nextFrameVector2D(random, frameA);
         FrameVector2D actual = new FrameVector2D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector2D expected = EuclidFrameRandomTools.nextFrameVector2D(random, frameA);
         FrameVector2D actual = new FrameVector2D(frameB, expected);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertFrameTuple3DEquals() throws Throwable
   {
      Random random = new Random(453453);
      ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame("frameA", random, worldFrame);
      ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame("frameB", random, worldFrame);

      String methodName = "assertFrameTuple3DEquals";
      Class<FrameTuple3DReadOnly> argumentsClass = FrameTuple3DReadOnly.class;

      {
         FrameVector3D expected = null;
         FrameVector3D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector3D expected = EuclidFrameRandomTools.nextFrameVector3D(random, frameA);
         FrameVector3D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector3D expected = null;
         FrameVector3D actual = EuclidFrameRandomTools.nextFrameVector3D(random, frameA);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector3D expected = EuclidFrameRandomTools.nextFrameVector3D(random, frameA);
         FrameVector3D actual = EuclidFrameRandomTools.nextFrameVector3D(random, frameA);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector3D expected = EuclidFrameRandomTools.nextFrameVector3D(random, frameA);
         FrameVector3D actual = new FrameVector3D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector3D expected = EuclidFrameRandomTools.nextFrameVector3D(random, frameA);
         FrameVector3D actual = new FrameVector3D(frameB, expected);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertFramePoint3DGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame("frameA", random, worldFrame);
      ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame("frameB", random, worldFrame);

      String methodName = "assertFramePoint3DGeometricallyEquals";
      Class<FramePoint3DReadOnly> argumentsClass = FramePoint3DReadOnly.class;

      {
         FramePoint3D expected = null;
         FramePoint3D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FramePoint3D expected = EuclidFrameRandomTools.nextFramePoint3D(random, frameA);
         FramePoint3D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FramePoint3D expected = null;
         FramePoint3D actual = EuclidFrameRandomTools.nextFramePoint3D(random, frameA);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FramePoint3D expected = EuclidFrameRandomTools.nextFramePoint3D(random, frameA);
         FramePoint3D actual = EuclidFrameRandomTools.nextFramePoint3D(random, frameA);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FramePoint3D expected = EuclidFrameRandomTools.nextFramePoint3D(random, frameA);
         FramePoint3D actual = new FramePoint3D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FramePoint3D expected = EuclidFrameRandomTools.nextFramePoint3D(random, frameA);
         FramePoint3D actual = new FramePoint3D(frameB, expected);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertFrameVector3DGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame("frameA", random, worldFrame);
      ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame("frameB", random, worldFrame);

      String methodName = "assertFrameVector3DGeometricallyEquals";
      Class<FrameVector3DReadOnly> argumentsClass = FrameVector3DReadOnly.class;

      {
         FrameVector3D expected = null;
         FrameVector3D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector3D expected = EuclidFrameRandomTools.nextFrameVector3D(random, frameA);
         FrameVector3D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector3D expected = null;
         FrameVector3D actual = EuclidFrameRandomTools.nextFrameVector3D(random, frameA);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector3D expected = EuclidFrameRandomTools.nextFrameVector3D(random, frameA);
         FrameVector3D actual = EuclidFrameRandomTools.nextFrameVector3D(random, frameA);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector3D expected = EuclidFrameRandomTools.nextFrameVector3D(random, frameA);
         FrameVector3D actual = new FrameVector3D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector3D expected = EuclidFrameRandomTools.nextFrameVector3D(random, frameA);
         FrameVector3D actual = new FrameVector3D(frameB, expected);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertFrameTuple4DEquals() throws Throwable
   {
      Random random = new Random(453453);
      ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame("frameA", random, worldFrame);
      ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame("frameB", random, worldFrame);

      String methodName = "assertFrameTuple4DEquals";
      Class<FrameTuple4DReadOnly> argumentsClass = FrameTuple4DReadOnly.class;

      {
         FrameVector4D expected = null;
         FrameVector4D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector4D expected = EuclidFrameRandomTools.nextFrameVector4D(random, frameA);
         FrameVector4D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector4D expected = null;
         FrameVector4D actual = EuclidFrameRandomTools.nextFrameVector4D(random, frameA);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector4D expected = EuclidFrameRandomTools.nextFrameVector4D(random, frameA);
         FrameVector4D actual = EuclidFrameRandomTools.nextFrameVector4D(random, frameA);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector4D expected = EuclidFrameRandomTools.nextFrameVector4D(random, frameA);
         FrameVector4D actual = new FrameVector4D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector4D expected = EuclidFrameRandomTools.nextFrameVector4D(random, frameA);
         FrameVector4D actual = new FrameVector4D(frameB, expected);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertFrameVector4DGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame("frameA", random, worldFrame);
      ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame("frameB", random, worldFrame);

      String methodName = "assertFrameVector4DGeometricallyEquals";
      Class<FrameVector4DReadOnly> argumentsClass = FrameVector4DReadOnly.class;

      {
         FrameVector4D expected = null;
         FrameVector4D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector4D expected = EuclidFrameRandomTools.nextFrameVector4D(random, frameA);
         FrameVector4D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector4D expected = null;
         FrameVector4D actual = EuclidFrameRandomTools.nextFrameVector4D(random, frameA);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector4D expected = EuclidFrameRandomTools.nextFrameVector4D(random, frameA);
         FrameVector4D actual = EuclidFrameRandomTools.nextFrameVector4D(random, frameA);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector4D expected = EuclidFrameRandomTools.nextFrameVector4D(random, frameA);
         FrameVector4D actual = new FrameVector4D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameVector4D expected = EuclidFrameRandomTools.nextFrameVector4D(random, frameA);
         FrameVector4D actual = new FrameVector4D(frameB, expected);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertFrameQuaternionGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame("frameA", random, worldFrame);
      ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame("frameB", random, worldFrame);

      String methodName = "assertFrameQuaternionGeometricallyEquals";
      Class<FrameQuaternionReadOnly> argumentsClass = FrameQuaternionReadOnly.class;

      {
         FrameQuaternion expected = null;
         FrameQuaternion actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameQuaternion expected = EuclidFrameRandomTools.nextFrameQuaternion(random, frameA);
         FrameQuaternion actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameQuaternion expected = null;
         FrameQuaternion actual = EuclidFrameRandomTools.nextFrameQuaternion(random, frameA);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameQuaternion expected = EuclidFrameRandomTools.nextFrameQuaternion(random, frameA);
         FrameQuaternion actual = EuclidFrameRandomTools.nextFrameQuaternion(random, frameA);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameQuaternion expected = EuclidFrameRandomTools.nextFrameQuaternion(random, frameA);
         FrameQuaternion actual = new FrameQuaternion(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         FrameQuaternion expected = EuclidFrameRandomTools.nextFrameQuaternion(random, frameA);
         FrameQuaternion actual = new FrameQuaternion(frameB, expected);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   private static void assertAssertionMethodsBehaveProperly(boolean failExpected, String methodName, Class<?> argumentsClass, Object expected, Object actual,
                                                            double epsilon)
         throws Throwable
   {
      EuclidCoreTestToolsTest.assertAssertionMethodsBehaveProperly(EuclidFrameTestTools.class, failExpected, methodName, argumentsClass, expected, actual,
                                                                   epsilon);
   }
}
