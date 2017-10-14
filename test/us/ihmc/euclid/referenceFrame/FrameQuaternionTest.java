package us.ihmc.euclid.referenceFrame;

public final class FrameQuaternionTest extends FrameQuaternionReadOnlyTest<FrameQuaternion>
{
   @Override
   public FrameQuaternion createTuple(ReferenceFrame referenceFrame, double x, double y, double z, double s)
   {
      FrameQuaternion ret = new FrameQuaternion(referenceFrame);
      ret.setUnsafe(x, y, z, s);
      return ret;
   }
}
