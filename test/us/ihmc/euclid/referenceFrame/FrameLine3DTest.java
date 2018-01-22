package us.ihmc.euclid.referenceFrame;

import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.Test;

import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;

public class FrameLine3DTest extends FrameLine3DReadOnlyTest<FrameLine3D>
{
   @Override
   public FrameLine3D createFrameLine(ReferenceFrame referenceFrame, Line3DReadOnly line)
   {
      return new FrameLine3D(referenceFrame, line);
   }

   @Test
   public void testConsistencyWithLine3D()
   {
      Random random = new Random(234235L);

      EuclidFrameAPITestTools.FrameTypeBuilder<? extends ReferenceFrameHolder> frameTypeBuilder = (frame, line) -> createFrameLine(frame, (Line3DReadOnly) line);
      EuclidFrameAPITestTools.GenericTypeBuilder framelessTypeBuilder = () -> EuclidGeometryRandomTools.nextLine3D(random);
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITestTools.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder, framelessTypeBuilder, methodFilter);
   }

   @Override
   @Test
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Map<String, Class<?>[]> framelessMethodsToIgnore = new HashMap<>();
      framelessMethodsToIgnore.put("set", new Class<?>[] {Line3D.class});
      framelessMethodsToIgnore.put("equals", new Class<?>[] {Line3D.class});
      framelessMethodsToIgnore.put("epsilonEquals", new Class<?>[] {Line3D.class, Double.TYPE});
      framelessMethodsToIgnore.put("geometricallyEquals", new Class<?>[] {Line3D.class, Double.TYPE});
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameLine3D.class, Line3D.class, true, 1, framelessMethodsToIgnore);
   }
}
