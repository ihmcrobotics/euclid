package us.ihmc.euclid.referenceFrame;

import org.junit.Test;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;
import java.util.function.Predicate;

public class FrameLineSegment2DTest extends FrameLineSegment2DReadOnlyTest<FrameLineSegment2D>
{
   @Override
   public FrameLineSegment2D createFrameLineSegment(ReferenceFrame referenceFrame, LineSegment2DReadOnly segment)
   {
      return new FrameLineSegment2D(referenceFrame, segment);
   }

   @Test
   public void testConsistencyWithLineSegment2D()
   {
      Random random = new Random(234235L);

      EuclidFrameAPITestTools.FrameTypeBuilder<? extends ReferenceFrameHolder> frameTypeBuilder = (frame, quaternion) -> createFrameLineSegment(frame, (LineSegment2DReadOnly) quaternion);
      EuclidFrameAPITestTools.GenericTypeBuilder framelessTypeBuilder = () -> createRandomLineSegment(random).getGeometryObject();
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") 
            && (!m.getReturnType().equals(Point2D[].class) && !(m.getParameterCount() > 0 && m.getParameterTypes()[0].equals(Point2D[].class)))
            && (!m.getReturnType().equals(Point2DReadOnly[].class) && !(m.getParameterCount() > 0 && m.getParameterTypes()[0].equals(Point2DReadOnly[].class)));
      EuclidFrameAPITestTools.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder, framelessTypeBuilder, methodFilter);
   }

   @Override
   @Test
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Map<String, Class<?>[]> framelessMethodsToIgnore = new HashMap<>();
      framelessMethodsToIgnore.put("set", new Class<?>[] {LineSegment2D.class});
      framelessMethodsToIgnore.put("epsilonEquals", new Class<?>[] {LineSegment2D.class, Double.TYPE});
      framelessMethodsToIgnore.put("geometricallyEquals", new Class<?>[] {LineSegment2D.class, Double.TYPE});
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameLineSegment2D.class, LineSegment2D.class, true, 1, framelessMethodsToIgnore);
   }
}
