package us.ihmc.euclid.referenceFrame;

import org.junit.Test;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;

import java.lang.reflect.Method;
import java.util.*;
import java.util.function.Predicate;

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

            fq2 = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, EuclidFrameRandomTools.generateRandomReferenceFrame(random));

            try
            {
               fq0.difference(fq1, fq2);
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
   }

   @Test
   public void testConsistencyWithQuaternion() {
      EuclidFrameAPITestTools.FrameTypeBuilder<? extends ReferenceFrameHolder> frameTypeBuilder = (frame, tuple) -> createTuple(frame, ((QuaternionReadOnly)tuple).getX(), ((QuaternionReadOnly)tuple).getY(), ((QuaternionReadOnly)tuple).getZ(), ((QuaternionReadOnly)tuple).getS());
      EuclidFrameAPITestTools.GenericTypeBuilder framelessTypeBuilder = () -> createRandomTuple(random).getGeometryObject();
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && Arrays.stream(m.getParameterTypes()).noneMatch(param -> param.getName().contains("4D") || (!param.getName().contains("Quaternion") && param.getName().contains("ReadOnly")));
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
}