package us.ihmc.euclid.referenceFrame;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

public abstract class FrameTuple2DReadOnlyTest<F extends FrameTuple2DReadOnly>
{
   public static final double EPSILON = 1.0e-15;

   public F createEmptyFrameTuple()
   {
      return createFrameTuple(ReferenceFrame.getWorldFrame(), 0.0, 0.0);
   }

   public final F createEmptyFrameTuple(ReferenceFrame referenceFrame)
   {
      return createFrameTuple(referenceFrame, 0.0, 0.0);
   }

   public F createRandomFrameTuple(Random random)
   {
      return createFrameTuple(ReferenceFrame.getWorldFrame(), random.nextDouble(), random.nextDouble());
   }

   public final F createRandomFrameTuple(Random random, ReferenceFrame referenceFrame)
   {
      return createFrameTuple(referenceFrame, random.nextDouble(), random.nextDouble());
   }

   public final F createFrameTuple(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple)
   {
      return createFrameTuple(referenceFrame, tuple.getX(), tuple.getY());
   }

   public final F createFrameTuple(F frameTuple)
   {
      return createFrameTuple(frameTuple.getReferenceFrame(), frameTuple);
   }

   public F createFrameTuple(double x, double y)
   {
      return createFrameTuple(ReferenceFrame.getWorldFrame(), x, y);
   }

   public abstract F createFrameTuple(ReferenceFrame referenceFrame, double x, double y);

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(621541L);
      double epsilon = 0.0;

      ReferenceFrame frame1 = ReferenceFrame.getWorldFrame();
      ReferenceFrame frame2 = EuclidFrameRandomTools.nextReferenceFrame(random);

      double x = random.nextDouble();
      double y = random.nextDouble();

      F tuple1 = createFrameTuple(frame1, x, y);
      F tuple2 = createFrameTuple(frame1, x, y);
      F tuple3 = createFrameTuple(frame2, x, y);
      F tuple4 = createFrameTuple(frame2, x, y);

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

      F tuple1 = createFrameTuple(frame1, x, y);
      F tuple2 = createFrameTuple(frame1, x, y);
      F tuple3 = createFrameTuple(frame2, x, y);
      F tuple4 = createFrameTuple(frame2, x, y);

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
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameTuple2DReadOnly.class, Tuple2DReadOnly.class, true);
   }
}
