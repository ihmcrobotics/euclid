package us.ihmc.euclid.referenceFrame;

import org.junit.Test;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

import java.lang.reflect.Method;
import java.util.Random;
import java.util.function.Predicate;

public class FrameLine2DTest extends FrameLine2DReadOnlyTest<FrameLine2D>
{
   @Override
   public FrameLine2D createFrameLine(ReferenceFrame referenceFrame, Line2DReadOnly line)
   {
      return new FrameLine2D(referenceFrame, line);
   }

   @Test
   public void testConsistencyWithLine2D()
   {
      Random random = new Random(234235L);

      EuclidFrameAPITestTools.FrameTypeBuilder<? extends ReferenceFrameHolder> frameTypeBuilder = (frame, line) -> createFrameLine(frame, (Line2DReadOnly) line);
      EuclidFrameAPITestTools.GenericTypeBuilder framelessTypeBuilder = () -> createRandomLine(random).getGeometryObject();
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && !m.getName().equals("setToZero") && !m.getName().equals("setToNaN")
            && (!m.getReturnType().equals(Point2D[].class) && !(m.getParameterCount() > 0 && m.getParameterTypes()[0].equals(Point2D[].class)))
            && (!m.getReturnType().equals(Point2DReadOnly[].class) && !(m.getParameterCount() > 0 && m.getParameterTypes()[0].equals(Point2DReadOnly[].class)));
      EuclidFrameAPITestTools.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder, framelessTypeBuilder, methodFilter);
   }

   @Override
   @Test
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Predicate<Method> framelessMethodsToIgnore = m -> !m.getName().equals("set")
            && !m.getName().equals("setPoint")
            && !m.getName().equals("setDirection")
            && !m.getName().equals("equals")
            && !m.getName().equals("epsilonEquals")
            && !m.getName().equals("geometricallyEquals");
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameLine2D.class, Line2D.class, true, 1, framelessMethodsToIgnore);
   }
}
