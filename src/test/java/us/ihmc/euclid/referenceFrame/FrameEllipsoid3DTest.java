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
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameBoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameEllipsoid3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameEllipsoid3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeRandomTools;
import us.ihmc.euclid.shape.primitives.Ellipsoid3D;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class FrameEllipsoid3DTest
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
      signaturesToIgnore.add(new MethodSignature("set", RigidBodyTransformReadOnly.class, Vector3DReadOnly.class));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);
      tester.assertOverloadingWithFrameObjects(FrameEllipsoid3DReadOnly.class, Ellipsoid3DReadOnly.class, false, 1, methodFilter);
      tester.assertOverloadingWithFrameObjects(FixedFrameEllipsoid3DBasics.class, Ellipsoid3DBasics.class, false, 1, methodFilter);

      signaturesToIgnore.add(new MethodSignature("set", Ellipsoid3D.class));
      signaturesToIgnore.add(new MethodSignature("epsilonEquals", Ellipsoid3D.class, Double.TYPE));
      signaturesToIgnore.add(new MethodSignature("geometricallyEquals", Ellipsoid3D.class, Double.TYPE));
      methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);
      tester.assertOverloadingWithFrameObjects(FrameEllipsoid3D.class, Ellipsoid3D.class, false, 1, methodFilter);
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
      tester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(EuclidFrameShapeRandomTools::nextFrameEllipsoid3D,
                                                                    methodFilter,
                                                                    EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Test
   public void testConsistencyWithEllipsoid3D()
   {
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && !m.getName().equals("epsilonEquals")
            && !m.getName().contains("IntermediateVariableSupplier") && !m.getName().equals("toString");
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameShapeAPIDefaultConfiguration());
      tester.assertFrameMethodsOfFrameHolderPreserveFunctionality((frame, ellipsoid) -> new FrameEllipsoid3D(frame, (Ellipsoid3D) ellipsoid),
                                                                  EuclidShapeRandomTools::nextEllipsoid3D,
                                                                  methodFilter,
                                                                  EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }

   @Test
   public void testSetMatchingFrame()
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameShapeAPIDefaultConfiguration());
      tester.assertSetMatchingFramePreserveFunctionality(EuclidFrameShapeRandomTools::nextFrameEllipsoid3D,
                                                         EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }

   @Test
   public void testSetIncludingFrame()
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameShapeAPIDefaultConfiguration());
      tester.assertSetIncludingFramePreserveFunctionality(EuclidFrameShapeRandomTools::nextFrameEllipsoid3D,
                                                          EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }
}
