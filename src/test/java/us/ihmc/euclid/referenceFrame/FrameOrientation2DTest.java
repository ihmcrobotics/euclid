package us.ihmc.euclid.referenceFrame;

import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.Test;

import us.ihmc.euclid.geometry.Orientation2D;
import us.ihmc.euclid.geometry.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameOrientation2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools.FrameTypeBuilder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;

public class FrameOrientation2DTest extends FrameOrientation2DReadOnlyTest<FrameOrientation2D>
{
   public static final int NUMBER_OF_ITERATIONS = 1000;
   public static final double EPSILON = 1.0e-15;

   @Override
   public FrameOrientation2D createFrameOrientation(ReferenceFrame referenceFrame, Orientation2DReadOnly orientation)
   {
      return new FrameOrientation2D(referenceFrame, orientation);
   }

   @Test
   public void testConsistencyWithOrientation2D()
   {
      Random random = new Random(234235L);

      FrameTypeBuilder<? extends ReferenceFrameHolder> frameTypeBuilder = (frame, pose) -> createFrameOrientation(frame, (Orientation2DReadOnly) pose);
      EuclidFrameAPITestTools.GenericTypeBuilder framelessTypeBuilder = () -> EuclidGeometryRandomTools.nextOrientation2D(random);
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

   @Test
   public void testSetMatchingFrame() throws Exception
   {
      Random random = new Random(544354);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);

         FrameOrientation2DReadOnly source = EuclidFrameRandomTools.nextFrameOrientation2D(random, sourceFrame);
         FixedFrameOrientation2DBasics actual = EuclidFrameRandomTools.nextFrameOrientation2D(random, destinationFrame);

         actual.setMatchingFrame(source);

         FrameOrientation2D expected = new FrameOrientation2D(source);
         expected.changeFrame(destinationFrame);

         EuclidGeometryTestTools.assertOrientation2DEquals(expected, actual, EPSILON);
      }
   }
}
