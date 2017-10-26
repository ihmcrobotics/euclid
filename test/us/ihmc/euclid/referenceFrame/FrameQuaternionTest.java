package us.ihmc.euclid.referenceFrame;

import org.junit.Test;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.QuaternionBasicsTest;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;
import java.util.function.Predicate;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public final class FrameQuaternionTest extends FrameQuaternionReadOnlyTest<FrameQuaternion>
{
   Random random = new Random(System.currentTimeMillis());

   @Override
   public FrameQuaternion createTuple(ReferenceFrame referenceFrame, double x, double y, double z, double s)
   {
      FrameQuaternion ret = new FrameQuaternion(referenceFrame);
      ret.setUnsafe(x, y, z, s);
      return ret;
   }

   @Test
   public void testReferenceFrameChecks()
   {
      ReferenceFrame frame;
      FrameQuaternion fq0;
      FrameQuaternion fq1;
      FrameQuaternion fq2;
      double alpha = 0.5;

      // setAndNormalize
      for (int i = 0; i < 100; ++i)
      {
         fq0 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame = EuclidFrameRandomTools.generateRandomReferenceFrame(random));

         if (random.nextDouble() > 0.5)
         {
            fq1 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame);

            try
            {
               fq0.setAndNormalize(fq1);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }
         }
         else
         {
            fq1 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, EuclidFrameRandomTools.generateRandomReferenceFrame(random));

            try
            {
               fq0.setAndNormalize(fq1);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }
         }
      }

      // setAndNegate
      for (int i = 0; i < 100; ++i)
      {
         fq0 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame = EuclidFrameRandomTools.generateRandomReferenceFrame(random));

         if (random.nextDouble() > 0.5)
         {
            fq1 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame);

            try
            {
               fq0.setAndNegate(fq1);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }
         }
         else
         {
            fq1 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, EuclidFrameRandomTools.generateRandomReferenceFrame(random));

            try
            {
               fq0.setAndNegate(fq1);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }
         }
      }

      // setAndConjugate
      for (int i = 0; i < 100; ++i)
      {
         fq0 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame = EuclidFrameRandomTools.generateRandomReferenceFrame(random));

         if (random.nextDouble() > 0.5)
         {
            fq1 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame);

            try
            {
               fq0.setAndConjugate(fq1);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }
         }
         else
         {
            fq1 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, EuclidFrameRandomTools.generateRandomReferenceFrame(random));

            try
            {
               fq0.setAndConjugate(fq1);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }
         }
      }

      // setAndInverse
      for (int i = 0; i < 100; ++i)
      {
         fq0 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame = EuclidFrameRandomTools.generateRandomReferenceFrame(random));

         if (random.nextDouble() > 0.5)
         {
            fq1 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame);

            try
            {
               fq0.setAndInverse(fq1);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }
         }
         else
         {
            fq1 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, EuclidFrameRandomTools.generateRandomReferenceFrame(random));

            try
            {
               fq0.setAndInverse(fq1);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }
         }
      }

      // interpolate
      for (int i = 0; i < 100; ++i)
      {
         fq0 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame = EuclidFrameRandomTools.generateRandomReferenceFrame(random));

         if (random.nextDouble() > 0.5)
         {
            fq1 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame);

            try
            {
               fq0.interpolate(fq1, alpha);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }

            fq2 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame);

            try
            {
               fq0.interpolate(fq1, fq2, alpha);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }
         }
         else
         {
            fq1 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, EuclidFrameRandomTools.generateRandomReferenceFrame(random));

            try
            {
               fq0.interpolate(fq1, alpha);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }

            fq2 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, EuclidFrameRandomTools.generateRandomReferenceFrame(random));

            try
            {
               fq0.interpolate(fq1, fq2, alpha);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }
         }
      }

      // multiply
      for (int i = 0; i < 100; ++i)
      {
         fq0 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame = EuclidFrameRandomTools.generateRandomReferenceFrame(random));

         if (random.nextDouble() > 0.5)
         {
            fq1 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame);

            try
            {
               fq0.multiply(fq1);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }

            fq2 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame);

            try
            {
               fq0.multiply(fq1, fq2);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }
         }
         else
         {
            fq1 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, EuclidFrameRandomTools.generateRandomReferenceFrame(random));

            try
            {
               fq0.multiply(fq1);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }

            fq2 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, EuclidFrameRandomTools.generateRandomReferenceFrame(random));

            try
            {
               fq0.multiply(fq1, fq2);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }
         }
      }

      // setEuler
      for (int i = 0; i < 100; ++i)
      {
         fq0 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame = EuclidFrameRandomTools.generateRandomReferenceFrame(random));

         if (random.nextDouble() > 0.5)
         {
            try
            {
               fq0.setEuler(EuclidFrameRandomTools.generateRandomFrameVector3D(random, frame));
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }
         }
         else
         {
            try
            {
               fq0.setEuler(EuclidFrameRandomTools.generateRandomFrameVector3D(random, EuclidFrameRandomTools.generateRandomReferenceFrame(random)));
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }
         }
      }

      // difference
      for (int i = 0; i < 100; ++i)
      {
         fq0 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame = EuclidFrameRandomTools.generateRandomReferenceFrame(random));

         if (random.nextDouble() > 0.5)
         {
            fq1 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame);

            fq2 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame);

            try
            {
               fq0.difference(fq1, fq2);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }
         }
         else
         {
            fq1 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, EuclidFrameRandomTools.generateRandomReferenceFrame(random));

            fq2 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame);

            try
            {
               fq0.difference(fq1, fq2);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }

            try
            {
               fq0.difference(fq2, fq1);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }
         }
      }

      // multiplyConjugateOther
      for (int i = 0; i < 100; ++i)
      {
         fq0 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame = EuclidFrameRandomTools.generateRandomReferenceFrame(random));

         if (random.nextDouble() > 0.5)
         {
            fq1 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame);

            try
            {
               fq0.multiplyConjugateOther(fq1);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }
         }
         else
         {
            fq1 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, EuclidFrameRandomTools.generateRandomReferenceFrame(random));

            try
            {
               fq0.multiplyConjugateOther(fq1);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }
         }
      }

      // multiplyConjugateThis
      for (int i = 0; i < 100; ++i)
      {
         fq0 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame = EuclidFrameRandomTools.generateRandomReferenceFrame(random));

         if (random.nextDouble() > 0.5)
         {
            fq1 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame);

            try
            {
               fq0.multiplyConjugateThis(fq1);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }
         }
         else
         {
            fq1 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, EuclidFrameRandomTools.generateRandomReferenceFrame(random));

            try
            {
               fq0.multiplyConjugateThis(fq1);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }
         }
      }

      // preMultiply
      for (int i = 0; i < 100; ++i)
      {
         fq0 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame = EuclidFrameRandomTools.generateRandomReferenceFrame(random));

         if (random.nextDouble() > 0.5)
         {
            fq1 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame);

            try
            {
               fq0.preMultiply(fq1);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }
         }
         else
         {
            fq1 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, EuclidFrameRandomTools.generateRandomReferenceFrame(random));

            try
            {
               fq0.preMultiply(fq1);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }
         }
      }

      // preMultiplyConjugateOther
      for (int i = 0; i < 100; ++i)
      {
         fq0 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame = EuclidFrameRandomTools.generateRandomReferenceFrame(random));

         if (random.nextDouble() > 0.5)
         {
            fq1 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame);

            try
            {
               fq0.preMultiplyConjugateOther(fq1);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }
         }
         else
         {
            fq1 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, EuclidFrameRandomTools.generateRandomReferenceFrame(random));

            try
            {
               fq0.preMultiplyConjugateOther(fq1);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }
         }
      }

      // preMultiplyConjugateThis
      for (int i = 0; i < 100; ++i)
      {
         fq0 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame = EuclidFrameRandomTools.generateRandomReferenceFrame(random));

         if (random.nextDouble() > 0.5)
         {
            fq1 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame);

            try
            {
               fq0.preMultiplyConjugateThis(fq1);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }
         }
         else
         {
            fq1 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, EuclidFrameRandomTools.generateRandomReferenceFrame(random));

            try
            {
               fq0.preMultiplyConjugateThis(fq1);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }
         }
      }

      // set
      for (int i = 0; i < 100; ++i)
      {
         fq0 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame = EuclidFrameRandomTools.generateRandomReferenceFrame(random));

         if (random.nextDouble() > 0.5)
         {
            fq1 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame);

            try
            {
               fq0.set(fq1);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }

            try
            {
               fq0.set(EuclidFrameRandomTools.generateRandomFrameVector3D(random, frame));
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }
         }
         else
         {
            fq1 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, EuclidFrameRandomTools.generateRandomReferenceFrame(random));

            try
            {
               fq0.set(fq1);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }

            try
            {
               fq0.set(EuclidFrameRandomTools.generateRandomFrameVector3D(random, EuclidFrameRandomTools.generateRandomReferenceFrame(random)));
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }
         }
      }

      // interpolate
      for (int i = 0; i < 100; ++i)
      {
         fq0 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame = EuclidFrameRandomTools.generateRandomReferenceFrame(random));
         fq2 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame);

         if (random.nextDouble() > 0.5)
         {
            fq1 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame);

            try
            {
               fq0.interpolate(fq1, fq2, 0.5);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }

            try
            {
               fq0.interpolate(fq2, fq1, 0.5);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }
         }
         else
         {
            fq1 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, EuclidFrameRandomTools.generateRandomReferenceFrame(random));

            try
            {
               fq0.interpolate(fq1, fq2, 0.5);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }

            try
            {
               fq0.interpolate(fq2, fq1, 0.5);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }
         }
      }
   }

   @Test
   public void testSetAndGetQuaternion() {
      for (int i = 0; i < 100; ++i) {
         Quaternion q = EuclidCoreRandomTools.generateRandomQuaternion(random);

         assertTrue(createTuple(q.getX(), q.getY(), q.getZ(), q.getS()).getQuaternion().epsilonEquals(q, getEpsilon()));
      }
   }

   @Test
   public void testConsistencyWithQuaternion() {
      EuclidFrameAPITestTools.FrameTypeBuilder<? extends ReferenceFrameHolder> frameTypeBuilder = (frame, tuple) -> createTuple(frame, ((QuaternionReadOnly)tuple).getX(), ((QuaternionReadOnly)tuple).getY(), ((QuaternionReadOnly)tuple).getZ(), ((QuaternionReadOnly)tuple).getS());
      EuclidFrameAPITestTools.GenericTypeBuilder framelessTypeBuilder = () -> createRandomTuple(random).getGeometryObject();
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode");
      EuclidFrameAPITestTools.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder, framelessTypeBuilder, methodFilter);
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
            return createEmptyTuple(referenceFrame);
         }

         @Override
         public FrameQuaternion createFrameGeometryObject(ReferenceFrame referenceFrame, Quaternion geometryObject)
         {
            return createTuple(referenceFrame, geometryObject.getX(), geometryObject.getY(), geometryObject.getZ(), geometryObject.getS());
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