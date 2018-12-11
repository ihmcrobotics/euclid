package us.ihmc.euclid.referenceFrame;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.tools.EuclidJUnitTools.*;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple4DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tuple4D.Tuple4DReadOnlyTest;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;

public abstract class FrameTuple4DReadOnlyTest<F extends FrameTuple4DReadOnly>
{
   public static final double EPSILON = 1.0e-15;

   public final F createEmptyFrameTuple()
   {
      return createEmptyFrameTuple(ReferenceFrame.getWorldFrame());
   }

   public F createEmptyFrameTuple(ReferenceFrame referenceFrame)
   {
      return createFrameTuple(referenceFrame, 0.0, 0.0, 0.0, 0.0);
   }

   public F createFrameTuple(ReferenceFrame referenceFrame, Tuple4DReadOnly tuple4dReadOnly)
   {
      return createFrameTuple(referenceFrame, tuple4dReadOnly.getX(), tuple4dReadOnly.getY(), tuple4dReadOnly.getZ(), tuple4dReadOnly.getS());
   }

   public final F createRandomFrameTuple(Random random)
   {
      return createRandomFrameTuple(random, ReferenceFrame.getWorldFrame());
   }

   public F createRandomFrameTuple(Random random, ReferenceFrame referenceFrame)
   {
      return createFrameTuple(referenceFrame, random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
   }

   public final F createFrameTuple(double x, double y, double z, double s)
   {
      return createFrameTuple(ReferenceFrame.getWorldFrame(), x, y, z, s);
   }

   public abstract F createFrameTuple(ReferenceFrame referenceFrame, double x, double y, double z, double s);

   @Test
   public void testTuple3DReadOnlyFeatures() throws Throwable
   {
      Tuple4DReadOnlyTest<F> test = new Tuple4DReadOnlyTest<F>()
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
         public F createTuple(double x, double y, double z, double s)
         {
            return createFrameTuple(x, y, z, s);
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
      double s = random.nextDouble();

      F tuple1 = createFrameTuple(frame1, x, y, z, s);
      F tuple2 = createFrameTuple(frame1, x, y, z, s);
      F tuple3 = createFrameTuple(frame2, x, y, z, s);
      F tuple4 = createFrameTuple(frame2, x, y, z, s);

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
      double s = random.nextDouble();

      F tuple1 = createFrameTuple(frame1, x, y, z, s);
      F tuple2 = createFrameTuple(frame1, x, y, z, s);
      F tuple3 = createFrameTuple(frame2, x, y, z, s);
      F tuple4 = createFrameTuple(frame2, x, y, z, s);

      assertTrue(tuple1.equals(tuple2));
      assertFalse(tuple1.equals(tuple3));
      assertFalse(tuple3.equals(tuple2));
      assertTrue(tuple3.equals(tuple4));

      assertTrue(tuple1.equals(tuple2));
      assertFalse(tuple1.equals(tuple3));
      assertFalse(tuple3.equals(tuple2));
      assertTrue(tuple3.equals(tuple4));
   }

   @Test
   public void testOverloading() throws Exception
   {
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameTuple4DReadOnly.class, Tuple4DReadOnly.class, true);
   }
}
