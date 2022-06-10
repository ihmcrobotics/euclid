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
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameCylinder3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCylinder3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeRandomTools;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class FrameCylinder3DTest
{
   @Test
   public void testAPIOverloading()
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameShapeAPIDefaultConfiguration());
      tester.assertOverloadingWithFrameObjects(FrameCylinder3DReadOnly.class, Cylinder3DReadOnly.class, false);

      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      signaturesToIgnore.add(new MethodSignature("setAxis", Vector3DReadOnly.class));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);
      tester.assertOverloadingWithFrameObjects(FixedFrameCylinder3DBasics.class, Cylinder3DBasics.class, false, 1, methodFilter);

      signaturesToIgnore.add(new MethodSignature("set", Cylinder3D.class));
      signaturesToIgnore.add(new MethodSignature("epsilonEquals", Cylinder3D.class, Double.TYPE));
      signaturesToIgnore.add(new MethodSignature("geometricallyEquals", Cylinder3D.class, Double.TYPE));
      methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);
      tester.assertOverloadingWithFrameObjects(FrameCylinder3D.class, Cylinder3D.class, false, 1, methodFilter);
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
      tester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(EuclidFrameShapeRandomTools::nextFrameCylinder3D,
                                                                    methodFilter,
                                                                    EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Test
   public void testConsistencyWithCylinder3D()
   {
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && !m.getName().equals("epsilonEquals") && !m.getName().equals("toString");
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameShapeAPIDefaultConfiguration());
      tester.assertFrameMethodsOfFrameHolderPreserveFunctionality((frame, cylinder) -> new FrameCylinder3D(frame, (Cylinder3D) cylinder),
                                                                  EuclidShapeRandomTools::nextCylinder3D,
                                                                  methodFilter,
                                                                  EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }

   @Test
   public void testSetMatchingFrame()
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameShapeAPIDefaultConfiguration());
      tester.assertSetMatchingFramePreserveFunctionality(EuclidFrameShapeRandomTools::nextFrameCylinder3D,
                                                         EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }

   @Test
   public void testSetIncludingFrame()
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameShapeAPIDefaultConfiguration());
      tester.assertSetIncludingFramePreserveFunctionality(EuclidFrameShapeRandomTools::nextFrameCylinder3D,
                                                          EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }
}
