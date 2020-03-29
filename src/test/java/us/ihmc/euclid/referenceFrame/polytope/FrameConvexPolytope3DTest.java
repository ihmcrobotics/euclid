package us.ihmc.euclid.referenceFrame.polytope;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Predicate;
import java.util.stream.Stream;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameShapeAPIDefaultConfiguration;
import us.ihmc.euclid.referenceFrame.api.MethodSignature;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameConvexPolytope3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameFace3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameHalfEdge3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameVertex3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeRandomTools;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;

public class FrameConvexPolytope3DTest
{
   @Test
   public void testAPIOverloading()
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameShapeAPIDefaultConfiguration());
      tester.assertOverloadingWithFrameObjects(FrameConvexPolytope3DReadOnly.class, ConvexPolytope3DReadOnly.class, false, 1);
      tester.assertOverloadingWithFrameObjects(FrameFace3DReadOnly.class, Face3DReadOnly.class, false, 1);
      tester.assertOverloadingWithFrameObjects(FrameHalfEdge3DReadOnly.class, HalfEdge3DReadOnly.class, false, 1);
      tester.assertOverloadingWithFrameObjects(FrameVertex3DReadOnly.class, Vertex3DReadOnly.class, false, 1);
   }

   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      Predicate<Method> methodFilter = m -> !m.getName().equals("equals");
      methodFilter = methodFilter.and(m -> !m.getName().equals("epsilonEquals"));
      methodFilter = methodFilter.and(m -> Stream.of(m.getParameterTypes()).noneMatch(type -> Vertex3DReadOnly.class.isAssignableFrom(type)));
      methodFilter = methodFilter.and(m -> !m.getName().equals("getBoundingBox")
            || !Arrays.equals(m.getParameterTypes(), new Class<?>[] {ReferenceFrame.class, FrameBoundingBox3DBasics.class}));
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameShapeAPIDefaultConfiguration());
      tester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(EuclidFrameShapeRandomTools::nextFrameConvexPolytope3D,
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
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameShapeAPIDefaultConfiguration());
      tester.assertFrameMethodsOfFrameHolderPreserveFunctionality((frame, convexPolytope) -> new FrameConvexPolytope3D(frame,
                                                                                                                       (ConvexPolytope3D) convexPolytope),
                                                                  EuclidShapeRandomTools::nextConvexPolytope3D,
                                                                  methodFilter,
                                                                  EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }
}
