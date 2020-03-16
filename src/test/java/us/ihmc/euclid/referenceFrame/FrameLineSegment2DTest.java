package us.ihmc.euclid.referenceFrame;

import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameLineSegment2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;

public class FrameLineSegment2DTest extends FrameLineSegment2DReadOnlyTest<FrameLineSegment2D>
{
   public static final double EPSILON = 1.0e-15;

   @Override
   public FrameLineSegment2D createFrameLineSegment(ReferenceFrame referenceFrame, LineSegment2DReadOnly segment)
   {
      return new FrameLineSegment2D(referenceFrame, segment);
   }

   @Test
   public void testConsistencyWithLineSegment2D()
   {
      EuclidFrameAPITestTools.FrameTypeBuilder<? extends ReferenceFrameHolder> frameTypeBuilder = (frame,
                                                                                                   quaternion) -> createFrameLineSegment(frame,
                                                                                                                                         (LineSegment2DReadOnly) quaternion);
      EuclidFrameAPITestTools.GenericTypeBuilder framelessTypeBuilder = EuclidGeometryRandomTools::nextLineSegment2D;
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITestTools.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder, framelessTypeBuilder, methodFilter);
   }

   @Override
   @Test
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Map<String, Class<?>[]> framelessMethodsToIgnore = new HashMap<>();
      framelessMethodsToIgnore.put("set", new Class<?>[] {LineSegment2D.class});
      framelessMethodsToIgnore.put("equals", new Class<?>[] {LineSegment2D.class});
      framelessMethodsToIgnore.put("epsilonEquals", new Class<?>[] {LineSegment2D.class, Double.TYPE});
      framelessMethodsToIgnore.put("geometricallyEquals", new Class<?>[] {LineSegment2D.class, Double.TYPE});
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameLineSegment2D.class, LineSegment2D.class, true, 1, framelessMethodsToIgnore);
   }

   @Test
   public void testSetMatchingFrame() throws Exception
   {
      Random random = new Random(544354);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);

         FrameLineSegment2DReadOnly source = EuclidFrameRandomTools.nextFrameLineSegment2D(random, sourceFrame);
         FixedFrameLineSegment2DBasics actual = EuclidFrameRandomTools.nextFrameLineSegment2D(random, destinationFrame);

         actual.setMatchingFrame(source);

         FrameLineSegment2D expected = new FrameLineSegment2D(source);
         expected.changeFrame(destinationFrame);

         EuclidGeometryTestTools.assertLineSegment2DEquals(expected, actual, EPSILON);
      }
   }
}
