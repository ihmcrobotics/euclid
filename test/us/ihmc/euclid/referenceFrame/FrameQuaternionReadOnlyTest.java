package us.ihmc.euclid.referenceFrame;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.Test;

import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple4DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Tuple4DReadOnlyTest;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public abstract class FrameQuaternionReadOnlyTest<T extends FrameQuaternionReadOnly> extends Tuple4DReadOnlyTest<T>
{
   public abstract T createFrameQuaternion(ReferenceFrame referenceFrame, QuaternionReadOnly quaternion);

   public abstract T createFrameQuaternion(ReferenceFrame referenceFrame, double x, double y, double z, double s);

   @Override
   public final T createEmptyTuple()
   {
      return createEmptyFrameQuaternion(ReferenceFrame.getWorldFrame());
   }

   @Override
   public final T createRandomTuple(Random random)
   {
      return createRandomFrameQuaternion(random, ReferenceFrame.getWorldFrame());
   }

   @Override
   public final T createTuple(double x, double y, double z, double s)
   {
      return createFrameQuaternion(ReferenceFrame.getWorldFrame(), x, y, z, s);
   }

   public final T createEmptyFrameQuaternion(ReferenceFrame referenceFrame)
   {
      return createFrameQuaternion(referenceFrame, new Quaternion());
   }

   public final T createRandomFrameQuaternion(Random random, ReferenceFrame referenceFrame)
   {
      return createFrameQuaternion(referenceFrame, EuclidCoreRandomTools.generateRandomQuaternion(random));
   }

   public final T createRandom2DFrameQuaternion(Random random, ReferenceFrame referenceFrame)
   {
      Quaternion quaternion = new Quaternion();
      quaternion.setToYawQuaternion(EuclidCoreRandomTools.generateRandomDouble(random, Math.PI));
      return createFrameQuaternion(referenceFrame, quaternion);
   }

   @Override
   public double getEpsilon()
   {
      return 1e-10;
   }

   @Test
   public void testOverloading() throws Exception
   {
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameQuaternionReadOnly.class, QuaternionReadOnly.class, true);
   }

   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      Random random = new Random(234);
      Predicate<Method> methodFilter = m -> !m.getName().contains("IncludingFrame") && !m.getName().equals("equals") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITestTools.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(frame -> createRandomFrameQuaternion(random, frame), false, true, methodFilter);
      EuclidFrameAPITestTools.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(frame -> createRandom2DFrameQuaternion(random, frame), false, true, methodFilter);
   }

   @Test
   public void testFrameTuple4DReadOnlyFeatures() throws Throwable
   {
      FrameTuple4DReadOnlyTest<FrameQuaternionReadOnly> frameTuple4DReadOnlyTest = new FrameTuple4DReadOnlyTest<FrameQuaternionReadOnly>()
      {

         @Override
         public FrameQuaternionReadOnly createTuple(ReferenceFrame referenceFrame, double x, double y, double z, double s)
         {
            return createFrameQuaternion(referenceFrame, x, y, z, s);
         }

         @Override
         public double getEpsilon()
         {
            return FrameQuaternionReadOnlyTest.this.getEpsilon();
         }
      };

      for (Method testMethod : frameTuple4DReadOnlyTest.getClass().getMethods())
      {
         if (!testMethod.getName().startsWith("test"))
            continue;
         if (!Modifier.isPublic(testMethod.getModifiers()))
            continue;
         if (Modifier.isStatic(testMethod.getModifiers()))
            continue;
//         if (testMethod.getName().equals("testGetGeometryObject"))
//            continue;

         try
         {
            testMethod.invoke(frameTuple4DReadOnlyTest);
         }
         catch (InvocationTargetException e)
         {
            throw e.getCause();
         }
      }
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      super.testEpsilonEquals();

      Random random = new Random(621541L);
      double epsilon = 0.0;

      ReferenceFrame frame1 = ReferenceFrame.getWorldFrame();
      ReferenceFrame frame2 = EuclidFrameRandomTools.generateRandomReferenceFrame(random);

      double x = random.nextDouble();
      double y = random.nextDouble();
      double z = random.nextDouble();
      double s = random.nextDouble();

      T tuple1 = createFrameQuaternion(frame1, x, y, z, s);
      T tuple2 = createFrameQuaternion(frame1, x, y, z, s);
      T tuple3 = createFrameQuaternion(frame2, x, y, z, s);
      T tuple4 = createFrameQuaternion(frame2, x, y, z, s);

      assertTrue(tuple1.epsilonEquals(tuple2, epsilon));
      assertFalse(tuple1.epsilonEquals(tuple3, epsilon));
      assertFalse(tuple3.epsilonEquals(tuple2, epsilon));
      assertTrue(tuple3.epsilonEquals(tuple4, epsilon));

      assertTrue(tuple1.epsilonEquals((FrameTuple4DReadOnly) tuple2, epsilon));
      assertFalse(tuple1.epsilonEquals((FrameTuple4DReadOnly) tuple3, epsilon));
      assertFalse(tuple3.epsilonEquals((FrameTuple4DReadOnly) tuple2, epsilon));
      assertTrue(tuple3.epsilonEquals((FrameTuple4DReadOnly) tuple4, epsilon));
   }

   @Test
   public void testEquals() throws Exception
   {
      super.testEquals();

      Random random = new Random(621541L);

      ReferenceFrame frame1 = ReferenceFrame.getWorldFrame();
      ReferenceFrame frame2 = EuclidFrameRandomTools.generateRandomReferenceFrame(random);

      double x = random.nextDouble();
      double y = random.nextDouble();
      double z = random.nextDouble();
      double s = random.nextDouble();

      T tuple1 = createFrameQuaternion(frame1, x, y, z, s);
      T tuple2 = createFrameQuaternion(frame1, x, y, z, s);
      T tuple3 = createFrameQuaternion(frame2, x, y, z, s);
      T tuple4 = createFrameQuaternion(frame2, x, y, z, s);

      assertTrue(tuple1.equals(tuple2));
      assertFalse(tuple1.equals(tuple3));
      assertFalse(tuple3.equals(tuple2));
      assertTrue(tuple3.equals(tuple4));
      
      assertTrue(tuple1.equals((FrameTuple4DReadOnly) tuple2));
      assertFalse(tuple1.equals((FrameTuple4DReadOnly) tuple3));
      assertFalse(tuple3.equals((FrameTuple4DReadOnly) tuple2));
      assertTrue(tuple3.equals((FrameTuple4DReadOnly) tuple4));
   }

}
