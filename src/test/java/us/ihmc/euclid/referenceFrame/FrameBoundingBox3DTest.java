package us.ihmc.euclid.referenceFrame;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.FrameTypeCopier;
import us.ihmc.euclid.referenceFrame.api.MethodSignature;
import us.ihmc.euclid.referenceFrame.api.RandomFramelessTypeBuilder;

public class FrameBoundingBox3DTest extends FrameBoundingBox3DReadOnlyTest<FrameBoundingBox3D>
{
   @Override
   public FrameBoundingBox3D createFrameBoundingBox(ReferenceFrame referenceFrame, BoundingBox3DReadOnly boundingBox)
   {
      return new FrameBoundingBox3D(referenceFrame, boundingBox);
   }

   @Test
   public void testConsistencyWithBoundingBox3D()
   {
      FrameTypeCopier frameTypeBuilder = (frame, boundingBox) -> createFrameBoundingBox(frame, (BoundingBox3DReadOnly) boundingBox);
      RandomFramelessTypeBuilder framelessTypeBuilder = EuclidGeometryRandomTools::nextBoundingBox3D;
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITester.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder, framelessTypeBuilder, methodFilter);
   }

   @Override
   @Test
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      signaturesToIgnore.add(new MethodSignature("set", BoundingBox3D.class));
      signaturesToIgnore.add(new MethodSignature("epsilonEquals", BoundingBox3D.class, Double.TYPE));
      signaturesToIgnore.add(new MethodSignature("geometricallyEquals", BoundingBox3D.class, Double.TYPE));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);
      
      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FrameBoundingBox3D.class, BoundingBox3D.class, false, 1, methodFilter);
   }
}
