package us.ihmc.euclid.referenceFrame;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class FrameConvexPolygon2DTest extends FrameConvexPolyong2DBasicsTest<FrameConvexPolygon2D>
{
   @Override
   public FrameConvexPolygon2D createFrameConvexPolygon2D(ReferenceFrame referenceFrame, List<? extends Point2DReadOnly> vertices)
   {
      return new FrameConvexPolygon2D(referenceFrame, vertices);
   }

   @Override
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Map<String, Class<?>[]> framelessMethodsToIgnore = new HashMap<>();
      framelessMethodsToIgnore.put("set", new Class<?>[] {ConvexPolygon2D.class});
      framelessMethodsToIgnore.put("epsilonEquals", new Class<?>[] {ConvexPolygon2D.class, Double.TYPE});
      framelessMethodsToIgnore.put("geometricallyEquals", new Class<?>[] {ConvexPolygon2D.class, Double.TYPE});
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameConvexPolygon2D.class, ConvexPolygon2D.class, true, 1, framelessMethodsToIgnore);
   }
}
