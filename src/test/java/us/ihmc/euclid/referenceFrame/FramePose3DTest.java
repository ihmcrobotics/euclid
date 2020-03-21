package us.ihmc.euclid.referenceFrame;

import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.FrameTypeCopier;
import us.ihmc.euclid.referenceFrame.api.MethodSignature;
import us.ihmc.euclid.referenceFrame.api.RandomFramelessTypeBuilder;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
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
      FrameTypeCopier frameTypeBuilder = (frame, pose) -> createFramePose(frame, (Pose3DReadOnly) pose);
      RandomFramelessTypeBuilder framelessTypeBuilder = EuclidGeometryRandomTools::nextPose3D;
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITester.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder,
                                                                                framelessTypeBuilder,
                                                                                methodFilter,
                                                                                EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);

      RandomFramelessTypeBuilder frameless2DTypeBuilder = (random) -> new Pose3D(createRandom2DFramePose(random, ReferenceFrame.getWorldFrame()));
      EuclidFrameAPITester.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder,
                                                                                frameless2DTypeBuilder,
                                                                                methodFilter,
                                                                                EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }

   @Override
   @Test
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      signaturesToIgnore.add(new MethodSignature("set", Pose3D.class));
      signaturesToIgnore.add(new MethodSignature("equals", Pose3D.class));
      signaturesToIgnore.add(new MethodSignature("epsilonEquals", Pose3D.class, Double.TYPE));
      signaturesToIgnore.add(new MethodSignature("geometricallyEquals", Pose3D.class, Double.TYPE));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);

      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FramePose3D.class, Pose3D.class, true, 1, methodFilter);
   }

   @Test
   public void testSetMatchingFrame() throws Exception
   {
      EuclidFrameAPITester.assertSetMatchingFramePreserveFunctionality(EuclidFrameRandomTools::nextFramePose3D,
                                                                       EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);

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
