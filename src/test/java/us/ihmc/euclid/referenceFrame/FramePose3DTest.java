package us.ihmc.euclid.referenceFrame;

import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;

public class FramePose3DTest extends FramePose3DReadOnlyTest<FramePose3D>
{
   public static final double EPSILON = 1.0e-15;

   @Override
   public FramePose3D createFramePose(ReferenceFrame referenceFrame, Pose3DReadOnly pose)
   {
      return new FramePose3D(referenceFrame, pose);
   }

   @Test
   public void testConsistencyWithPose3D()
   {
      Random random = new Random(234235L);

      EuclidFrameAPITestTools.FrameTypeBuilder<? extends ReferenceFrameHolder> frameTypeBuilder = (frame, pose) -> createFramePose(frame,
                                                                                                                                   (Pose3DReadOnly) pose);
      EuclidFrameAPITestTools.GenericTypeBuilder framelessTypeBuilder = () -> EuclidGeometryRandomTools.nextPose3D(random);
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITestTools.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder, framelessTypeBuilder, methodFilter);

      EuclidFrameAPITestTools.GenericTypeBuilder frameless2DTypeBuilder = () -> new Pose3D(createRandom2DFramePose(random, ReferenceFrame.getWorldFrame()));
      EuclidFrameAPITestTools.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder, frameless2DTypeBuilder, methodFilter);
   }

   @Override
   @Test
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Map<String, Class<?>[]> framelessMethodsToIgnore = new HashMap<>();
      framelessMethodsToIgnore.put("set", new Class<?>[] {Pose3D.class});
      framelessMethodsToIgnore.put("equals", new Class<?>[] {Pose3D.class});
      framelessMethodsToIgnore.put("epsilonEquals", new Class<?>[] {Pose3D.class, Double.TYPE});
      framelessMethodsToIgnore.put("geometricallyEquals", new Class<?>[] {Pose3D.class, Double.TYPE});
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FramePose3D.class, Pose3D.class, true, 1, framelessMethodsToIgnore);
   }

   @Test
   public void testSetMatchingFrame() throws Exception
   {
      Random random = new Random(544354);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

         FramePose3DReadOnly source = EuclidFrameRandomTools.nextFramePose3D(random, sourceFrame);
         FixedFramePose3DBasics actual = EuclidFrameRandomTools.nextFramePose3D(random, destinationFrame);

         actual.setMatchingFrame(source);

         FramePose3D expected = new FramePose3D(source);
         expected.changeFrame(destinationFrame);

         EuclidGeometryTestTools.assertPose3DEquals(expected, actual, EPSILON);
      }
   }
}
