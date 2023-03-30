package us.ihmc.euclid.referenceFrame;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameShapeAPIDefaultConfiguration;
import us.ihmc.euclid.referenceFrame.api.MethodSignature;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameShape3DPoseBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DPoseReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeRandomTools;
import us.ihmc.euclid.shape.primitives.Shape3DPose;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DPoseBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DPoseReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;

public class FixedFrameShape3DPoseTest
{
   @BeforeEach
   public void disableNameRestriction()
   {
      ReferenceFrame.getWorldFrame().setNameRestrictionLevel(FrameNameRestrictionLevel.NONE);
   }

   @Test
   public void testAPIOverloading()
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameShapeAPIDefaultConfiguration());
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);
      tester.assertOverloadingWithFrameObjects(FrameShape3DPoseReadOnly.class, Shape3DPoseReadOnly.class, false, 1, methodFilter);

      methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);
      tester.assertOverloadingWithFrameObjects(FixedFrameShape3DPoseBasics.class, Shape3DPoseBasics.class, false, 1, methodFilter);

      signaturesToIgnore.add(new MethodSignature("set", Shape3DPose.class));
      signaturesToIgnore.add(new MethodSignature("epsilonEquals", Shape3DPose.class, Double.TYPE));
      signaturesToIgnore.add(new MethodSignature("geometricallyEquals", Shape3DPose.class, Double.TYPE));
      methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);
      tester.assertOverloadingWithFrameObjects(FixedFrameShape3DPose.class, Shape3DPose.class, false, 1, methodFilter);
   }

   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      Predicate<Method> methodFilter = m -> !m.getName().equals("equals");
      methodFilter = methodFilter.and(m -> !m.getName().equals("epsilonEquals"));
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameShapeAPIDefaultConfiguration());
      tester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(EuclidFrameShapeRandomTools::nextFixedFrameShape3DPose,
                                                                    methodFilter,
                                                                    EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);

   }

   @Test
   public void testConsistencyWithShape3DPose()
   {
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && !m.getName().equals("epsilonEquals") && !m.getName().equals("toString");
      methodFilter = methodFilter.and(m -> !m.getName().contains("ChangeListener"));
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameShapeAPIDefaultConfiguration());
      tester.assertFrameMethodsOfFrameHolderPreserveFunctionality((frame, shape3DPose) -> new FixedFrameShape3DPose(() -> frame, (Shape3DPose) shape3DPose),
                                                                  EuclidShapeRandomTools::nextShape3DPose,
                                                                  methodFilter,
                                                                  EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }

   @Test
   public void testSetMatchingFrame()
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameShapeAPIDefaultConfiguration());
      tester.assertSetMatchingFramePreserveFunctionality(EuclidFrameShapeRandomTools::nextFixedFrameShape3DPose,
                                                         EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }
}
