package us.ihmc.euclid.referenceFrame;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tuple3D.Tuple3DReadOnlyTest;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public abstract class FrameTuple3DReadOnlyTest<T extends FrameTuple3DReadOnly> extends Tuple3DReadOnlyTest<T>
{
   @Override
   public final T createEmptyTuple()
   {
      return createTuple(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);
   }

   public final T createEmptyTuple(ReferenceFrame referenceFrame)
   {
      return createTuple(referenceFrame, 0.0, 0.0, 0.0);
   }

   @Override
   public final T createRandomTuple(Random random)
   {
      return createTuple(ReferenceFrame.getWorldFrame(), random.nextDouble(), random.nextDouble(), random.nextDouble());
   }

   @Override
   public final T createTuple(double x, double y, double z)
   {
      return createTuple(ReferenceFrame.getWorldFrame(), x, y, z);
   }

   public abstract T createTuple(ReferenceFrame referenceFrame, double x, double y, double z);

   @Override
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

      T tuple1 = createTuple(frame1, x, y, z);
      T tuple2 = createTuple(frame1, x, y, z);
      T tuple3 = createTuple(frame2, x, y, z);
      T tuple4 = createTuple(frame2, x, y, z);

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
      ReferenceFrame frame2 = EuclidFrameRandomTools.generateRandomReferenceFrame(random);

      double x = random.nextDouble();
      double y = random.nextDouble();
      double z = random.nextDouble();

      T tuple1 = createTuple(frame1, x, y, z);
      T tuple2 = createTuple(frame1, x, y, z);
      T tuple3 = createTuple(frame2, x, y, z);
      T tuple4 = createTuple(frame2, x, y, z);

      assertTrue(tuple1.equals(tuple2));
      assertFalse(tuple1.equals(tuple3));
      assertFalse(tuple3.equals(tuple2));
      assertTrue(tuple3.equals(tuple4));
   }

   @Test
   public void testOverloading() throws Exception
   {
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameTuple3DReadOnly.class, Tuple3DReadOnly.class, true);
   }
}
