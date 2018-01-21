package us.ihmc.euclid.referenceFrame;

import java.lang.reflect.Method;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.Test;

import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

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
      EuclidFrameAPITestTools.GenericTypeBuilder framelessTypeBuilder = () -> createRandomLine(random).getGeometryObject();
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && !m.getName().equals("setToZero") && !m.getName().equals("setToNaN")
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
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameLine3D.class, Line3D.class, true, 1, framelessMethodsToIgnore);
   }
}
