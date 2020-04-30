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
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoxPolytope3DView;
import us.ihmc.euclid.referenceFrame.polytope.FrameConvexPolytope3D;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeRandomTools;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.BoxPolytope3DView;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class FrameBoxPolytope3DTest
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
         tester.assertOverloadingWithFrameObjects(FrameBoxPolytope3DView.class, BoxPolytope3DView.class, false, 1, methodFilter);
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
      tester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(FrameBoxPolytope3DTest::nextFrameBoxPolytope3DView,
                                                                    methodFilter,
                                                                    EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Test
   public void testConsistencyWithBox3D()
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
      tester.assertFrameMethodsOfFrameHolderPreserveFunctionality((frame, boxPolytope) -> copy(frame, (BoxPolytope3DView) boxPolytope),
                                                                  FrameBoxPolytope3DTest::nextBoxPolytope3DView,
                                                                  methodFilter,
                                                                  EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }

   private static BoxPolytope3DView nextBoxPolytope3DView(Random random)
   {
      return EuclidShapeRandomTools.nextBox3D(random).asConvexPolytope();
   }

   private static FrameBoxPolytope3DView nextFrameBoxPolytope3DView(Random random, ReferenceFrame referenceFrame)
   {
      return EuclidFrameShapeRandomTools.nextFrameBox3D(random, referenceFrame).asConvexPolytope();
   }

   private static FrameBoxPolytope3DView copy(ReferenceFrame referenceFrame, BoxPolytope3DView boxPolytope3DView)
   {
      return new FrameBox3D(referenceFrame, (Box3DReadOnly) boxPolytope3DView.copy()).asConvexPolytope();
   }
}
