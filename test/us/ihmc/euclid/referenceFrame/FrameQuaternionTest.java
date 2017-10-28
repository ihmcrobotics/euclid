package us.ihmc.euclid.referenceFrame;

import static org.junit.Assert.assertTrue;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;
import java.util.function.Predicate;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple4DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools.FrameTypeBuilder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools.GenericTypeBuilder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.QuaternionBasicsTest;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;

public final class FrameQuaternionTest extends FrameQuaternionReadOnlyTest<FrameQuaternion>
{
   public static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   public static final double EPSILON = 1e-10;

   @Override
   public FrameQuaternion createFrameQuaternion(ReferenceFrame referenceFrame, QuaternionReadOnly quaternion)
   {
      return new FrameQuaternion(referenceFrame, quaternion);
   }

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(435345);

      { // Test FrameQuaternion()
         FrameQuaternion frameQuaternion = new FrameQuaternion();
         assertTrue(frameQuaternion.referenceFrame == worldFrame);
         EuclidCoreTestTools.assertQuaternionIsSetToZero(frameQuaternion);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameQuaternion(ReferenceFrame referenceFrame)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         FrameQuaternion frameQuaternion = new FrameQuaternion(randomFrame);
         assertTrue(frameQuaternion.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertQuaternionIsSetToZero(frameQuaternion);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameQuaternion(ReferenceFrame referenceFrame, double x, double y, double z, double s)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         Quaternion randomQuaternion = EuclidCoreRandomTools.generateRandomQuaternion(random);
         FrameQuaternion frameQuaternion = new FrameQuaternion(randomFrame, randomQuaternion.getX(), randomQuaternion.getY(), randomQuaternion.getZ(),
                                                               randomQuaternion.getS());
         assertTrue(frameQuaternion.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple4DEquals(randomQuaternion, frameQuaternion, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameQuaternion(ReferenceFrame referenceFrame, double[] quaternionArray)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         Quaternion randomQuaternion = EuclidCoreRandomTools.generateRandomQuaternion(random);
         double[] array = new double[4];
         randomQuaternion.get(array);
         FrameQuaternion frameQuaternion = new FrameQuaternion(randomFrame, array);
         assertTrue(frameQuaternion.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple4DEquals(randomQuaternion, frameQuaternion, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameQuaternion(ReferenceFrame referenceFrame, DenseMatrix64F matrix)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         Quaternion randomQuaternion = EuclidCoreRandomTools.generateRandomQuaternion(random);
         DenseMatrix64F denseMatrix = new DenseMatrix64F(4, 1);
         randomQuaternion.get(denseMatrix);
         FrameQuaternion frameQuaternion = new FrameQuaternion(randomFrame, denseMatrix);
         assertTrue(frameQuaternion.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple4DEquals(randomQuaternion, frameQuaternion, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameQuaternion(ReferenceFrame referenceFrame, QuaternionReadOnly quaternionReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         QuaternionReadOnly randomQuaternion = EuclidCoreRandomTools.generateRandomQuaternion(random);
         FrameQuaternion frameQuaternion = new FrameQuaternion(randomFrame, randomQuaternion);
         assertTrue(frameQuaternion.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple4DEquals(randomQuaternion, frameQuaternion, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameQuaternion(ReferenceFrame referenceFrame, Tuple4DReadOnly tuple4DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         Tuple4DReadOnly randomTuple = EuclidCoreRandomTools.generateRandomVector4D(random);
         Quaternion expectedQuaternion = new Quaternion(randomTuple);
         FrameQuaternion frameQuaternion = new FrameQuaternion(randomFrame, randomTuple);
         EuclidCoreTestTools.assertTuple4DEquals(expectedQuaternion, frameQuaternion, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameQuaternion(ReferenceFrame referenceFrame, RotationMatrixReadOnly rotationMatrix)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         RotationMatrixReadOnly randomRotationMatrix = EuclidCoreRandomTools.generateRandomRotationMatrix(random);
         Quaternion expectedQuaternion = new Quaternion(randomRotationMatrix);
         FrameQuaternion frameQuaternion = new FrameQuaternion(randomFrame, randomRotationMatrix);
         EuclidCoreTestTools.assertTuple4DEquals(expectedQuaternion, frameQuaternion, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameQuaternion(ReferenceFrame referenceFrame, AxisAngleReadOnly axisAngle)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         AxisAngleReadOnly randomAxisAngle = EuclidCoreRandomTools.generateRandomAxisAngle(random);
         Quaternion expectedQuaternion = new Quaternion(randomAxisAngle);
         FrameQuaternion frameQuaternion = new FrameQuaternion(randomFrame, randomAxisAngle);
         EuclidCoreTestTools.assertTuple4DEquals(expectedQuaternion, frameQuaternion, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameQuaternion(ReferenceFrame referenceFrame, Vector3DReadOnly rotationVector)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         Vector3DReadOnly randomRotationVector = EuclidCoreRandomTools.generateRandomVector3D(random);
         Quaternion expectedQuaternion = new Quaternion(randomRotationVector);
         FrameQuaternion frameQuaternion = new FrameQuaternion(randomFrame, randomRotationVector);
         EuclidCoreTestTools.assertTuple4DEquals(expectedQuaternion, frameQuaternion, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test FrameQuaternion(FrameQuaternionReadOnly other)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         FrameQuaternion randomFrameQuaternion = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, randomFrame);
         FrameQuaternion frameQuaternion = new FrameQuaternion(randomFrameQuaternion);
         assertTrue(frameQuaternion.referenceFrame == randomFrame);
         EuclidCoreTestTools.assertTuple4DEquals(randomFrameQuaternion, frameQuaternion, EPSILON);
         EuclidFrameTestTools.assertFrameTuple4DEquals(randomFrameQuaternion, frameQuaternion, EPSILON);
      }
   }

   // TODO This test should become obsolete once FrameVector4D is implemented and added to EuclidFrameAPITestTools.
   @Test
   public void testSetAndNormalize() throws Exception
   {
      Random random = new Random(435345L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         ReferenceFrame frameA = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         ReferenceFrame frameB = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         Tuple4DReadOnly randomTuple = EuclidCoreRandomTools.generateRandomVector4D(random);
         FrameTuple4DReadOnly randomFrameTuple = createFrameTuple4DReadOnly(frameA, randomTuple);
         Quaternion expectedQuaternion = new Quaternion();
         FrameQuaternion frameQuaternion = new FrameQuaternion(frameA);

         expectedQuaternion.setAndNormalize(randomTuple);
         frameQuaternion.setAndNormalize(randomFrameTuple);

         assertTrue(frameQuaternion.referenceFrame == frameA);
         EuclidCoreTestTools.assertTuple4DEquals(expectedQuaternion, frameQuaternion, EPSILON);

         try
         {
            frameQuaternion.setAndNormalize(createFrameTuple4DReadOnly(frameB, randomTuple));
            throw new AssertionError("Should have thrown an exception.");
         }
         catch (ReferenceFrameMismatchException e)
         {
            // good
         }
      }
   }

   private FrameTuple4DReadOnly createFrameTuple4DReadOnly(ReferenceFrame referenceFrame, Tuple4DReadOnly tuple4D)
   {
      return new FrameTuple4DReadOnly()
      {
         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrame;
         }

         @Override
         public double getX()
         {
            return tuple4D.getX();
         }

         @Override
         public double getY()
         {
            return tuple4D.getY();
         }

         @Override
         public double getZ()
         {
            return tuple4D.getZ();
         }

         @Override
         public double getS()
         {
            return tuple4D.getS();
         }
      };
   }

   @Override
   public FrameQuaternion createFrameQuaternion(ReferenceFrame referenceFrame, double x, double y, double z, double s)
   {
      FrameQuaternion frameQuaternion = new FrameQuaternion(referenceFrame);
      frameQuaternion.setUnsafe(x, y, z, s);
      return frameQuaternion;
   }

   @Test
   public void testConsistencyWithQuaternion()
   {
      Random random = new Random(234235L);

      FrameTypeBuilder<? extends ReferenceFrameHolder> frameTypeBuilder = (frame, quaternion) -> createFrameQuaternion(frame, (QuaternionReadOnly) quaternion);
      GenericTypeBuilder framelessTypeBuilder = () -> createRandomTuple(random).getGeometryObject();
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode");
      EuclidFrameAPITestTools.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder, framelessTypeBuilder, methodFilter);

      GenericTypeBuilder frameless2DTypeBuilder = () -> createRandom2DFrameQuaternion(random, ReferenceFrame.getWorldFrame()).getGeometryObject();
      EuclidFrameAPITestTools.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder, frameless2DTypeBuilder, methodFilter);
   }

   @Test
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Map<String, Class<?>[]> framelessMethodsToIgnore = new HashMap<>();
      framelessMethodsToIgnore.put("set", new Class<?>[] {Quaternion.class});
      framelessMethodsToIgnore.put("epsilonEquals", new Class<?>[] {Quaternion.class, Double.TYPE});
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameQuaternion.class, Quaternion.class, true, 1, framelessMethodsToIgnore);
   }

   @Test
   public void testGetQuaternion()
   {
      Random random = new Random(43535);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Quaternion expected = EuclidCoreRandomTools.generateRandomQuaternion(random);
         FrameQuaternion frameQuaternion = new FrameQuaternion(worldFrame, expected);
         QuaternionReadOnly actual = frameQuaternion.getQuaternion();
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, EPSILON);
         EuclidCoreTestTools.assertTuple4DEquals(frameQuaternion, actual, EPSILON);
      }
   }

   @Test
   public void testFrameGeometryObjectFeatures() throws Throwable
   {
      FrameGeometryObjectTest<FrameQuaternion, Quaternion> frameGeometryObjectTest = new FrameGeometryObjectTest<FrameQuaternion, Quaternion>()
      {
         @Override
         public Quaternion createEmptyGeometryObject()
         {
            return createEmptyTuple().getGeometryObject();
         }

         @Override
         public Quaternion createRandomGeometryObject(Random random)
         {
            return createRandomTuple(random).getGeometryObject();
         }

         @Override
         public FrameQuaternion createEmptyFrameGeometryObject(ReferenceFrame referenceFrame)
         {
            return createEmptyFrameQuaternion(referenceFrame);
         }

         @Override
         public FrameQuaternion createFrameGeometryObject(ReferenceFrame referenceFrame, Quaternion geometryObject)
         {
            return createFrameQuaternion(referenceFrame, geometryObject);
         }

         @Override
         public FrameQuaternion createRandomFrameGeometryObject(Random random, ReferenceFrame referenceFrame)
         {
            return EuclidFrameRandomTools.generateRandomFrameQuaternion(random, referenceFrame);
         }
      };

      for (Method testMethod : frameGeometryObjectTest.getClass().getMethods())
      {
         if (!testMethod.getName().startsWith("test"))
            continue;
         if (!Modifier.isPublic(testMethod.getModifiers()))
            continue;
         if (Modifier.isStatic(testMethod.getModifiers()))
            continue;
         // The following are due to normalization altering values during the createTuple() calls
         if (testMethod.getName().equals("testSetFromReferenceFrame"))
            continue;
         if (testMethod.getName().equals("testChangeFrame"))
            continue;
         if (testMethod.getName().equals("testGetGeometryObject"))
            continue;

         try
         {
            testMethod.invoke(frameGeometryObjectTest);
         }
         catch (InvocationTargetException e)
         {
            throw e.getCause();
         }
      }
   }

   @Test
   public void testQuaternionBasicsFeatures() throws Exception
   {
      QuaternionBasicsTest<FrameQuaternion> quaternionBasicsTest = new QuaternionBasicsTest<FrameQuaternion>()
      {
         @Override
         public FrameQuaternion createEmptyTuple()
         {
            return new FrameQuaternion();
         }

         @Override
         public FrameQuaternion createTuple(double v, double v1, double v2, double v3)
         {
            FrameQuaternion ret = new FrameQuaternion(ReferenceFrame.getWorldFrame());
            ret.setUnsafe(v, v1, v2, v3);
            return ret;
         }

         @Override
         public FrameQuaternion createRandomTuple(Random random)
         {
            return EuclidFrameRandomTools.generateRandomFrameQuaternion(random, ReferenceFrame.getWorldFrame());
         }

         @Override
         public double getEpsilon()
         {
            return 1e-10;
         }
      };

      for (Method testMethod : quaternionBasicsTest.getClass().getMethods())
      {
         if (!testMethod.getName().startsWith("test"))
            continue;
         if (!Modifier.isPublic(testMethod.getModifiers()))
            continue;
         if (Modifier.isStatic(testMethod.getModifiers()))
            continue;

         testMethod.invoke(quaternionBasicsTest);
      }
   }
}