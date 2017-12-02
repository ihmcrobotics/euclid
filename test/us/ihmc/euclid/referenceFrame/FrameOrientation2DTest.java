package us.ihmc.euclid.referenceFrame;

import org.junit.Test;
import us.ihmc.euclid.geometry.Orientation2D;
import us.ihmc.euclid.geometry.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;

import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;
import java.util.function.Predicate;

public class FrameOrientation2DTest extends FrameOrientation2DReadOnlyTest<FrameOrientation2D>
{
   @Override
   public FrameOrientation2D createFrameOrientation(ReferenceFrame referenceFrame, Orientation2DReadOnly orientation)
   {
      return new FrameOrientation2D(referenceFrame, orientation);
   }

   @Test
   public void testConsistencyWithOrientation2D()
   {
      Random random = new Random(234235L);

      EuclidFrameAPITestTools.FrameTypeBuilder<? extends ReferenceFrameHolder> frameTypeBuilder = (frame, pose) -> createFrameOrientation(frame,
                                                                                                                                          (Orientation2DReadOnly) pose);
      EuclidFrameAPITestTools.GenericTypeBuilder framelessTypeBuilder = () -> createRandomOrientation(random).getGeometryObject();
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITestTools.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder, framelessTypeBuilder, methodFilter);
   }

   @Override
   @Test
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Map<String, Class<?>[]> framelessMethodsToIgnore = new HashMap<>();
      framelessMethodsToIgnore.put("set", new Class<?>[] {Orientation2D.class});
      framelessMethodsToIgnore.put("equals", new Class<?>[] {Orientation2D.class});
      framelessMethodsToIgnore.put("epsilonEquals", new Class<?>[] {Orientation2D.class, Double.TYPE});
      framelessMethodsToIgnore.put("geometricallyEquals", new Class<?>[] {Orientation2D.class, Double.TYPE});
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameOrientation2D.class, Orientation2D.class, true, 1, framelessMethodsToIgnore);
   }
}
