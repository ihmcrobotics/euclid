package us.ihmc.euclid.referenceFrame;

import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;

public class FramePose2DTest extends FramePose2DReadOnlyTest<FramePose2D>
{
   public static final double EPSILON = 1.0e-15;

   @Override
   public FramePose2D createFramePose(ReferenceFrame referenceFrame, Pose2DReadOnly pose)
   {
      return new FramePose2D(referenceFrame, pose);
   }

   @Test
   public void testConsistencyWithPose2D()
   {
      EuclidFrameAPITester.FrameTypeBuilder<? extends ReferenceFrameHolder> frameTypeBuilder = (frame, pose) -> createFramePose(frame,
                                                                                                                                   (Pose2DReadOnly) pose);
      EuclidFrameAPITester.GenericTypeBuilder framelessTypeBuilder = EuclidGeometryRandomTools::nextPose2D;
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITester.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder, framelessTypeBuilder, methodFilter);
   }

   @Override
   @Test
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Map<String, Class<?>[]> framelessMethodsToIgnore = new HashMap<>();
      framelessMethodsToIgnore.put("set", new Class<?>[] {Pose2D.class});
      framelessMethodsToIgnore.put("equals", new Class<?>[] {Pose2D.class});
      framelessMethodsToIgnore.put("epsilonEquals", new Class<?>[] {Pose2D.class, Double.TYPE});
      framelessMethodsToIgnore.put("geometricallyEquals", new Class<?>[] {Pose2D.class, Double.TYPE});
      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FramePose2D.class, Pose2D.class, true, 1, framelessMethodsToIgnore);
   }

   @Test
   public void testSetMatchingFrame() throws Exception
   {
      Random random = new Random(544354);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);

         FramePose2DReadOnly source = EuclidFrameRandomTools.nextFramePose2D(random, sourceFrame);
         FixedFramePose2DBasics actual = EuclidFrameRandomTools.nextFramePose2D(random, destinationFrame);

         actual.setMatchingFrame(source);

         FramePose2D expected = new FramePose2D(source);
         expected.changeFrame(destinationFrame);

         EuclidGeometryTestTools.assertPose2DEquals(expected, actual, EPSILON);
      }
   }
}
