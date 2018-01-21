package us.ihmc.euclid.referenceFrame;

import java.lang.reflect.Method;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.Test;

import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class FrameLineSegment3DTest extends FrameLineSegment3DReadOnlyTest<FrameLineSegment3D>
{
   @Override
   public FrameLineSegment3D createFrameLineSegment(ReferenceFrame referenceFrame, LineSegment3DReadOnly segment)
   {
      return new FrameLineSegment3D(referenceFrame, segment);
   }

   @Test
   public void testConsistencyWithLineSegment2D()
   {
      Random random = new Random(234235L);

      EuclidFrameAPITestTools.FrameTypeBuilder<? extends ReferenceFrameHolder> frameTypeBuilder = (frame, quaternion) -> createFrameLineSegment(frame, (LineSegment3DReadOnly) quaternion);
      EuclidFrameAPITestTools.GenericTypeBuilder framelessTypeBuilder = () -> createRandomLineSegment(random).getGeometryObject();
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode")
            && (!m.getReturnType().equals(Point3D[].class) && !(m.getParameterCount() > 0 && m.getParameterTypes()[0].equals(Point3D[].class)))
            && (!m.getReturnType().equals(Point3DReadOnly[].class) && !(m.getParameterCount() > 0 && m.getParameterTypes()[0].equals(Point3DReadOnly[].class)));
      EuclidFrameAPITestTools.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder, framelessTypeBuilder, methodFilter);
   }

   @Override
   @Test
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Predicate<Method> framelessMethodsToIgnore = m -> !m.getName().equals("set")
            && !m.getName().equals("equals")
            && !m.getName().equals("epsilonEquals")
            && !m.getName().equals("geometricallyEquals");
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameLineSegment3D.class, LineSegment3D.class, true, 1, framelessMethodsToIgnore);
   }
}
