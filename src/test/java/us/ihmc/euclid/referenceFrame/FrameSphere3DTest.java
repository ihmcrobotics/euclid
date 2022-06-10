package us.ihmc.euclid.referenceFrame;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameShapeAPIDefaultConfiguration;
import us.ihmc.euclid.referenceFrame.api.MethodSignature;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameBoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameSphere3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameSphere3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeRandomTools;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;

public class FrameSphere3DTest
{
   @Test
   public void testAPIOverloading()
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameShapeAPIDefaultConfiguration());
      tester.assertOverloadingWithFrameObjects(FrameSphere3DReadOnly.class, Sphere3DReadOnly.class, false);
      tester.assertOverloadingWithFrameObjects(FixedFrameSphere3DBasics.class, Sphere3DBasics.class, false);

      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      signaturesToIgnore.add(new MethodSignature("set", Sphere3D.class));
      signaturesToIgnore.add(new MethodSignature("epsilonEquals", Sphere3D.class, Double.TYPE));
      signaturesToIgnore.add(new MethodSignature("geometricallyEquals", Sphere3D.class, Double.TYPE));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);
      tester.assertOverloadingWithFrameObjects(FrameSphere3D.class, Sphere3D.class, false, 1, methodFilter);
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
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameShapeAPIDefaultConfiguration());
      tester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(EuclidFrameShapeRandomTools::nextFrameSphere3D,
                                                                    methodFilter,
                                                                    EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Test
   public void testConsistencyWithSphere3D()
   {
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && !m.getName().equals("epsilonEquals") && !m.getName().equals("toString");
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameShapeAPIDefaultConfiguration());
      tester.assertFrameMethodsOfFrameHolderPreserveFunctionality((frame, sphere) -> new FrameSphere3D(frame, (Sphere3D) sphere),
                                                                  EuclidShapeRandomTools::nextSphere3D,
                                                                  methodFilter,
                                                                  EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }

   @Test
   public void testSetMatchingFrame()
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameShapeAPIDefaultConfiguration());
      tester.assertSetMatchingFramePreserveFunctionality(EuclidFrameShapeRandomTools::nextFrameSphere3D, EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }

   @Test
   public void testSetIncludingFrame()
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameShapeAPIDefaultConfiguration());
      tester.assertSetIncludingFramePreserveFunctionality(EuclidFrameShapeRandomTools::nextFrameSphere3D,
                                                          EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }
}
