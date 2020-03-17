package us.ihmc.euclid.referenceFrame;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITestTools;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tuple3D.Tuple3DReadOnlyTest;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public abstract class FrameTuple3DReadOnlyTest<F extends FrameTuple3DReadOnly>
{
   public static final double EPSILON = 1.0e-15;

   public final F createEmptyFrameTuple()
   {
      return createFrameTuple(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);
   }

   public final F createEmptyFrameTuple(ReferenceFrame referenceFrame)
   {
      return createFrameTuple(referenceFrame, 0.0, 0.0, 0.0);
   }

   public final F createFrameTuple(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple)
   {
      return createFrameTuple(referenceFrame, tuple.getX(), tuple.getY(), tuple.getZ());
   }

   public final F createRandomFrameTuple(Random random)
   {
      return createFrameTuple(ReferenceFrame.getWorldFrame(), random.nextDouble(), random.nextDouble(), random.nextDouble());
   }

   public final F createRandomFrameTuple(Random random, ReferenceFrame referenceFrame)
   {
      return createFrameTuple(referenceFrame, random.nextDouble(), random.nextDouble(), random.nextDouble());
   }

   public final F createFrameTuple(double x, double y, double z)
   {
      return createFrameTuple(ReferenceFrame.getWorldFrame(), x, y, z);
   }

   public abstract F createFrameTuple(ReferenceFrame referenceFrame, double x, double y, double z);

   @Test
   public void testTuple3DReadOnlyFeatures() throws Throwable
   {
      Tuple3DReadOnlyTest<F> test = new Tuple3DReadOnlyTest<F>()
      {
         @Override
         public F createEmptyTuple()
         {
            return createEmptyFrameTuple();
         }

         @Override
         public F createRandomTuple(Random random)
         {
            return createRandomFrameTuple(random);
         }

         @Override
         public F createTuple(double x, double y, double z)
         {
            return createFrameTuple(x, y, z);
         }

         @Override
         public double getEpsilon()
         {
            return EPSILON;
         }
      };

      for (Method testMethod : test.getClass().getMethods())
      {
         if (!testMethod.getName().startsWith("test"))
            continue;
         if (!Modifier.isPublic(testMethod.getModifiers()))
            continue;
         if (Modifier.isStatic(testMethod.getModifiers()))
            continue;

         try
         {
            testMethod.invoke(test);
         }
         catch (InvocationTargetException e)
         {
            throw e.getTargetException();
         }
      }
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(621541L);
      double epsilon = 0.0;

      ReferenceFrame frame1 = ReferenceFrame.getWorldFrame();
      ReferenceFrame frame2 = EuclidFrameRandomTools.nextReferenceFrame(random);

      double x = random.nextDouble();
      double y = random.nextDouble();
      double z = random.nextDouble();

      F tuple1 = createFrameTuple(frame1, x, y, z);
      F tuple2 = createFrameTuple(frame1, x, y, z);
      F tuple3 = createFrameTuple(frame2, x, y, z);
      F tuple4 = createFrameTuple(frame2, x, y, z);

      assertTrue(tuple1.epsilonEquals(tuple2, epsilon));
      assertFalse(tuple1.epsilonEquals(tuple3, epsilon));
      assertFalse(tuple3.epsilonEquals(tuple2, epsilon));
      assertTrue(tuple3.epsilonEquals(tuple4, epsilon));
   }

   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(621541L);

      ReferenceFrame frame1 = ReferenceFrame.getWorldFrame();
      ReferenceFrame frame2 = EuclidFrameRandomTools.nextReferenceFrame(random);

      double x = random.nextDouble();
      double y = random.nextDouble();
      double z = random.nextDouble();

      F tuple1 = createFrameTuple(frame1, x, y, z);
      F tuple2 = createFrameTuple(frame1, x, y, z);
      F tuple3 = createFrameTuple(frame2, x, y, z);
      F tuple4 = createFrameTuple(frame2, x, y, z);

      assertTrue(tuple1.equals(tuple2));
      assertFalse(tuple1.equals(tuple3));
      assertFalse(tuple3.equals(tuple2));
      assertTrue(tuple3.equals(tuple4));

      Object tuple2AsObject = tuple2;
      Object tuple3AsObject = tuple3;
      Object tuple4AsObject = tuple4;
      assertTrue(tuple1.equals(tuple2AsObject));
      assertFalse(tuple1.equals(tuple3AsObject));
      assertFalse(tuple3.equals(tuple2AsObject));
      assertTrue(tuple3.equals(tuple4AsObject));
   }

   @Test
   public void testOverloading() throws Exception
   {
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameTuple3DReadOnly.class, Tuple3DReadOnly.class, true);
   }
}
