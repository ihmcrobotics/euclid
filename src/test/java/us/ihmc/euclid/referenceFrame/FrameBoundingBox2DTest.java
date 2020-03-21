package us.ihmc.euclid.referenceFrame;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.FrameTypeCopier;
import us.ihmc.euclid.referenceFrame.api.MethodSignature;
import us.ihmc.euclid.referenceFrame.api.RandomFramelessTypeBuilder;

public class FrameBoundingBox2DTest extends FrameBoundingBox2DReadOnlyTest<FrameBoundingBox2D>
{
   @Override
   public FrameBoundingBox2D createFrameBoundingBox(ReferenceFrame referenceFrame, BoundingBox2DReadOnly boundingBox)
   {
      return new FrameBoundingBox2D(referenceFrame, boundingBox);
   }

   @Test
   public void testConsistencyWithBoundingBox2D()
   {
      FrameTypeCopier frameTypeBuilder = (frame, boundingBox) -> createFrameBoundingBox(frame, (BoundingBox2DReadOnly) boundingBox);
      RandomFramelessTypeBuilder framelessTypeBuilder = EuclidGeometryRandomTools::nextBoundingBox2D;
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITester.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder,
                                                                                framelessTypeBuilder,
                                                                                methodFilter,
                                                                                EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }

   @Override
   @Test
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      signaturesToIgnore.add(new MethodSignature("set", BoundingBox2D.class));
      signaturesToIgnore.add(new MethodSignature("epsilonEquals", BoundingBox2D.class, Double.TYPE));
      signaturesToIgnore.add(new MethodSignature("geometricallyEquals", BoundingBox2D.class, Double.TYPE));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);

      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FrameBoundingBox2D.class, BoundingBox2D.class, false, 1, methodFilter);
   }
}
