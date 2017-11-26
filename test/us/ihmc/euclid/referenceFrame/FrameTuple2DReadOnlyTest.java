package us.ihmc.euclid.referenceFrame;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tuple2D.Tuple2DReadOnlyTest;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

public abstract class FrameTuple2DReadOnlyTest<T extends FrameTuple2DReadOnly> extends Tuple2DReadOnlyTest<T>
{
   @Override
   public T createEmptyTuple()
   {
      return createTuple(ReferenceFrame.getWorldFrame(), 0.0, 0.0);
   }

   public T createEmptyTuple(ReferenceFrame referenceFrame)
   {
      return createTuple(referenceFrame, 0.0, 0.0);
   }

   @Override
   public T createRandomTuple(Random random)
   {
      return createTuple(ReferenceFrame.getWorldFrame(), random.nextDouble(), random.nextDouble());
   }

   @Override
   public T createTuple(double x, double y)
   {
      return createTuple(ReferenceFrame.getWorldFrame(), x, y);
   }

   public abstract T createTuple(ReferenceFrame referenceFrame, double x, double y);

   @Override
   @Test
   public void testEpsilonEquals() throws Exception
   {
      super.testEpsilonEquals();

      Random random = new Random(621541L);
      double epsilon = 0.0;

      ReferenceFrame frame1 = ReferenceFrame.getWorldFrame();
      ReferenceFrame frame2 = EuclidFrameRandomTools.nextReferenceFrame(random);

      double x = random.nextDouble();
      double y = random.nextDouble();

      T tuple1 = createTuple(frame1, x, y);
      T tuple2 = createTuple(frame1, x, y);
      T tuple3 = createTuple(frame2, x, y);
      T tuple4 = createTuple(frame2, x, y);

      assertTrue(tuple1.epsilonEquals(tuple2, epsilon));
      assertFalse(tuple1.epsilonEquals(tuple3, epsilon));
      assertFalse(tuple3.epsilonEquals(tuple2, epsilon));
      assertTrue(tuple3.epsilonEquals(tuple4, epsilon));
   }

   @Override
   @Test
   public void testEquals() throws Exception
   {
      super.testEquals();

      Random random = new Random(621541L);

      ReferenceFrame frame1 = ReferenceFrame.getWorldFrame();
      ReferenceFrame frame2 = EuclidFrameRandomTools.nextReferenceFrame(random);

      double x = random.nextDouble();
      double y = random.nextDouble();

      T tuple1 = createTuple(frame1, x, y);
      T tuple2 = createTuple(frame1, x, y);
      T tuple3 = createTuple(frame2, x, y);
      T tuple4 = createTuple(frame2, x, y);

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
