package us.ihmc.euclid.referenceFrame;

import org.junit.Test;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public final class FrameQuaternionTest extends FrameQuaternionReadOnlyTest<FrameQuaternion>
{
   @Override
   public FrameQuaternion createTuple(ReferenceFrame referenceFrame, double x, double y, double z, double s)
   {
      FrameQuaternion ret = new FrameQuaternion(referenceFrame);
      ret.setUnsafe(x, y, z, s);
      return ret;
   }

   @Test
   public void testOverloading() throws Exception
   {
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameQuaternionReadOnly.class, QuaternionReadOnly.class, true);
   }
}
