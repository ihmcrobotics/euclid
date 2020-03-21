package us.ihmc.euclid.referenceFrame;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.matrix.RotationScaleMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationScaleMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.MethodSignature;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameShape3DPoseBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DPoseReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeRandomTools;
import us.ihmc.euclid.shape.primitives.Shape3DPose;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DPoseBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DPoseReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class FrameShape3DPoseTest extends FrameShapeSetupTest
{
   @Test
   public void testAPIOverloading()
   {
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      signaturesToIgnore.add(new MethodSignature("get", RotationScaleMatrix.class, Tuple3DBasics.class));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);
      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FrameShape3DPoseReadOnly.class, Shape3DPoseReadOnly.class, false, 1, methodFilter);

      signaturesToIgnore.add(new MethodSignature("set", RotationScaleMatrixReadOnly.class, Tuple3DReadOnly.class));
      methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);
      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FixedFrameShape3DPoseBasics.class, Shape3DPoseBasics.class, false, 1, methodFilter);

      signaturesToIgnore.add(new MethodSignature("set", Shape3DPose.class));
      signaturesToIgnore.add(new MethodSignature("epsilonEquals", Shape3DPose.class, Double.TYPE));
      signaturesToIgnore.add(new MethodSignature("geometricallyEquals", Shape3DPose.class, Double.TYPE));
      methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);
      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FrameShape3DPose.class, Shape3DPose.class, false, 1, methodFilter);
   }

   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      Predicate<Method> methodFilter = m -> !m.getName().equals("setIncludingFrame");
      methodFilter = methodFilter.and(m -> !m.getName().equals("setMatchingFrame"));
      methodFilter = methodFilter.and(m -> !m.getName().equals("equals"));
      methodFilter = methodFilter.and(m -> !m.getName().equals("epsilonEquals"));
      methodFilter = methodFilter.and(m -> !m.getName().equals("getBoundingBox")
            || !Arrays.equals(m.getParameterTypes(), new Class<?>[] {ReferenceFrame.class, FrameBoundingBox3DBasics.class}));
      EuclidFrameAPITester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(EuclidFrameShapeRandomTools::nextFrameShape3DPose,
                                                                                  methodFilter,
                                                                                  EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }
}
