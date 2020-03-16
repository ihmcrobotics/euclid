package us.ihmc.euclid.referenceFrame;

import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameLineSegment3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;

public class FrameLineSegment3DTest extends FrameLineSegment3DReadOnlyTest<FrameLineSegment3D>
{
   public static final double EPSILON = 1.0e-15;

   @Override
   public FrameLineSegment3D createFrameLineSegment(ReferenceFrame referenceFrame, LineSegment3DReadOnly segment)
   {
      return new FrameLineSegment3D(referenceFrame, segment);
   }

   @Test
   public void testConsistencyWithLineSegment2D()
   {
      EuclidFrameAPITestTools.FrameTypeBuilder<? extends ReferenceFrameHolder> frameTypeBuilder = (frame,
                                                                                                   quaternion) -> createFrameLineSegment(frame,
                                                                                                                                         (LineSegment3DReadOnly) quaternion);
      EuclidFrameAPITestTools.GenericTypeBuilder framelessTypeBuilder = EuclidGeometryRandomTools::nextLineSegment3D;
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITestTools.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder, framelessTypeBuilder, methodFilter);
   }

   @Override
   @Test
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Map<String, Class<?>[]> framelessMethodsToIgnore = new HashMap<>();
      framelessMethodsToIgnore.put("set", new Class<?>[] {LineSegment3D.class});
      framelessMethodsToIgnore.put("equals", new Class<?>[] {LineSegment3D.class});
      framelessMethodsToIgnore.put("epsilonEquals", new Class<?>[] {LineSegment3D.class, Double.TYPE});
      framelessMethodsToIgnore.put("geometricallyEquals", new Class<?>[] {LineSegment3D.class, Double.TYPE});
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameLineSegment3D.class, LineSegment3D.class, true, 1, framelessMethodsToIgnore);
   }

   @Test
   public void testSetMatchingFrame() throws Exception
   {
      Random random = new Random(544354);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

         FrameLineSegment3DReadOnly source = EuclidFrameRandomTools.nextFrameLineSegment3D(random, sourceFrame);
         FixedFrameLineSegment3DBasics actual = EuclidFrameRandomTools.nextFrameLineSegment3D(random, destinationFrame);

         actual.setMatchingFrame(source);

         FrameLineSegment3D expected = new FrameLineSegment3D(source);
         expected.changeFrame(destinationFrame);

         EuclidGeometryTestTools.assertLineSegment3DEquals(expected, actual, EPSILON);
      }
   }
}
