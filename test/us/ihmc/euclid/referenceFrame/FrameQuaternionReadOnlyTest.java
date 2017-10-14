package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.tuple4D.Tuple4DReadOnlyTest;

import java.util.Random;

public abstract class FrameQuaternionReadOnlyTest<T extends FrameQuaternion> extends Tuple4DReadOnlyTest<T>
{
   @Override
   public final T createEmptyTuple()
   {
      return createTuple(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0, 0.0);
   }

   public final T createEmptyTuple(ReferenceFrame referenceFrame)
   {
      return createTuple(referenceFrame, 0.0, 0.0, 0.0, 0.0);
   }

   @Override
   public final T createRandomTuple(Random random)
   {
      return createTuple(ReferenceFrame.getWorldFrame(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
   }

   @Override
   public final T createTuple(double x, double y, double z, double s)
   {
      return createTuple(ReferenceFrame.getWorldFrame(), x, y, z, s);
   }

   public abstract T createQuaternionUnsafe(ReferenceFrame referenceFrame, double x, double y, double z, double s);

   public T createQuaternionUnsafe(double x, double y, double z, double s) {
      return createQuaternionUnsafe(ReferenceFrame.getWorldFrame(), x, y, z, s);
   }

   public abstract T createTuple(ReferenceFrame referenceFrame, double x, double y, double z, double s);

   @Override
   public double getEpsilon()
   {
      return 1e-10;
   }
}
