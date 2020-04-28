package us.ihmc.euclid.referenceFrame;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Predicate;
import java.util.stream.Stream;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameShapeAPIDefaultConfiguration;
import us.ihmc.euclid.referenceFrame.api.MethodSignature;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameBoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRampPolytope3DView;
import us.ihmc.euclid.referenceFrame.polytope.FrameConvexPolytope3D;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeRandomTools;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.RampPolytope3DView;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class FrameRampPolytope3DTest
{
   @Test
   public void testAPIOverloading()
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameShapeAPIDefaultConfiguration());

      {
         List<MethodSignature> signaturesToIgnore = new ArrayList<>();
         signaturesToIgnore.add(new MethodSignature("getSupportingVertex", Vertex3DReadOnly.class, Vector3DReadOnly.class));
         Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);
         methodFilter = methodFilter.and(m -> m.getParameterTypes().equals(new Class<?>[] {Axis3D.class}));
         tester.assertOverloadingWithFrameObjects(FrameRampPolytope3DView.class, RampPolytope3DView.class, false, 1, methodFilter);
      }
   }

   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      signaturesToIgnore.add(new MethodSignature("getBoundingBox", FixedFrameBoundingBox3DBasics.class));
      signaturesToIgnore.add(new MethodSignature("getBoundingBox", ReferenceFrame.class, FrameBoundingBox3DBasics.class));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);
      methodFilter = methodFilter.and(m -> !m.getName().equals("equals"));
      methodFilter = methodFilter.and(m -> !m.getName().equals("epsilonEquals"));
      methodFilter = methodFilter.and(m -> Stream.of(m.getParameterTypes()).noneMatch(type -> Vertex3DReadOnly.class.isAssignableFrom(type)));
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameShapeAPIDefaultConfiguration());
      tester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(FrameRampPolytope3DTest::nextFrameRampPolytope3DView,
                                                                    methodFilter,
                                                                    EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Test
   public void testConsistencyWithRamp3D()
   {
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      signaturesToIgnore.add(new MethodSignature("hashCode"));
      signaturesToIgnore.add(new MethodSignature("epsilonEquals", FrameConvexPolytope3D.class, double.class));
      signaturesToIgnore.add(new MethodSignature("geometricallyEquals", FrameConvexPolytope3D.class, double.class));
      signaturesToIgnore.add(new MethodSignature("set", FrameConvexPolytope3D.class));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);
      methodFilter = methodFilter.and(m -> Stream.of(m.getParameterTypes()).noneMatch(type -> Vertex3DReadOnly.class.isAssignableFrom(type)));
      methodFilter = methodFilter.and(m -> m.getName().equals("othogonalProjectionCopy"));
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameShapeAPIDefaultConfiguration());
      tester.assertFrameMethodsOfFrameHolderPreserveFunctionality((frame, rampPolytope) -> copy(frame, (RampPolytope3DView) rampPolytope),
                                                                  FrameRampPolytope3DTest::nextRampPolytope3DView,
                                                                  methodFilter,
                                                                  EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }

   private static RampPolytope3DView nextRampPolytope3DView(Random random)
   {
      return EuclidShapeRandomTools.nextRamp3D(random).asConvexPolytope();
   }

   private static FrameRampPolytope3DView nextFrameRampPolytope3DView(Random random, ReferenceFrame referenceFrame)
   {
      return EuclidFrameShapeRandomTools.nextFrameRamp3D(random, referenceFrame).asConvexPolytope();
   }

   private static FrameRampPolytope3DView copy(ReferenceFrame referenceFrame, RampPolytope3DView rampPolytope3DView)
   {
      return new FrameRamp3D(referenceFrame, (Ramp3DReadOnly) rampPolytope3DView.copy()).asConvexPolytope();
   }
}
