package us.ihmc.euclid.referenceFrame;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.MethodSignature;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePointShape3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePointShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeRandomTools;
import us.ihmc.euclid.shape.primitives.PointShape3D;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DReadOnly;

public class FramePointShape3DTest extends FrameShapeSetupTest
{
   @Test
   public void testAPIOverloading()
   {
      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FramePointShape3DReadOnly.class, PointShape3DReadOnly.class, false);
      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FixedFramePointShape3DBasics.class, PointShape3DBasics.class, false);

      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      signaturesToIgnore.add(new MethodSignature("set", PointShape3D.class));
      signaturesToIgnore.add(new MethodSignature("epsilonEquals", PointShape3D.class, Double.TYPE));
      signaturesToIgnore.add(new MethodSignature("geometricallyEquals", PointShape3D.class, Double.TYPE));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);
      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FramePointShape3D.class, PointShape3D.class, false, 1, methodFilter);
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
      EuclidFrameAPITester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(EuclidFrameShapeRandomTools::nextFramePointShape3D,
                                                                                  methodFilter,
                                                                                  EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }
}
