package us.ihmc.euclid.referenceFrame;

import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;

public class FrameBoundingBox3DTest extends FrameBoundingBox3DReadOnlyTest<FrameBoundingBox3D>
{
   @Override
   public FrameBoundingBox3D createFrameBoundingBox(ReferenceFrame referenceFrame, BoundingBox3DReadOnly boundingBox)
   {
      return new FrameBoundingBox3D(referenceFrame, boundingBox);
   }

   @Test
   public void testConsistencyWithBoundingBox3D()
   {
      Random random = new Random(234235L);

      EuclidFrameAPITestTools.FrameTypeBuilder<? extends ReferenceFrameHolder> frameTypeBuilder = (frame,
                                                                                                   boundingBox) -> createFrameBoundingBox(frame,
                                                                                                                                          (BoundingBox3DReadOnly) boundingBox);
      EuclidFrameAPITestTools.GenericTypeBuilder framelessTypeBuilder = () -> EuclidGeometryRandomTools.nextBoundingBox3D(random);
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITestTools.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder, framelessTypeBuilder, methodFilter);
   }

   @Override
   @Test
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Map<String, Class<?>[]> framelessMethodsToIgnore = new HashMap<>();
      framelessMethodsToIgnore.put("set", new Class<?>[] {BoundingBox3D.class});
      framelessMethodsToIgnore.put("epsilonEquals", new Class<?>[] {BoundingBox3D.class, Double.TYPE});
      framelessMethodsToIgnore.put("geometricallyEquals", new Class<?>[] {BoundingBox3D.class, Double.TYPE});
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameBoundingBox3D.class, BoundingBox3D.class, false, 1, framelessMethodsToIgnore);
   }
}
