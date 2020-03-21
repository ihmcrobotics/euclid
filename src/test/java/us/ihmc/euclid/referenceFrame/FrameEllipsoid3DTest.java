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
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameEllipsoid3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameEllipsoid3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeRandomTools;
import us.ihmc.euclid.shape.primitives.Ellipsoid3D;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;

public class FrameEllipsoid3DTest extends FrameShapeSetupTest
{
   @Test
   public void testAPIOverloading()
   {
      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FrameEllipsoid3DReadOnly.class, Ellipsoid3DReadOnly.class, false);
      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FixedFrameEllipsoid3DBasics.class, Ellipsoid3DBasics.class, false);

      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      signaturesToIgnore.add(new MethodSignature("set", Ellipsoid3D.class));
      signaturesToIgnore.add(new MethodSignature("epsilonEquals", Ellipsoid3D.class, Double.TYPE));
      signaturesToIgnore.add(new MethodSignature("geometricallyEquals", Ellipsoid3D.class, Double.TYPE));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);
      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FrameEllipsoid3D.class, Ellipsoid3D.class, false, 1, methodFilter);
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
      EuclidFrameAPITester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(EuclidFrameShapeRandomTools::nextFrameEllipsoid3D,
                                                                                  methodFilter,
                                                                                  EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }
}
