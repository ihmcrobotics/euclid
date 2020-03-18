package us.ihmc.euclid.referenceFrame;

import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.FrameTypeCopier;
import us.ihmc.euclid.referenceFrame.api.MethodSignature;
import us.ihmc.euclid.referenceFrame.api.RandomFramelessTypeBuilder;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;

public class FrameLine3DTest extends FrameLine3DReadOnlyTest<FrameLine3D>
{
   public static final double EPSILON = 1.0e-15;

   @Override
   public FrameLine3D createFrameLine(ReferenceFrame referenceFrame, Line3DReadOnly line)
   {
      return new FrameLine3D(referenceFrame, line);
   }

   @Test
   public void testConsistencyWithLine3D()
   {
      FrameTypeCopier frameTypeBuilder = (frame, line) -> createFrameLine(frame, (Line3DReadOnly) line);
      RandomFramelessTypeBuilder framelessTypeBuilder = EuclidGeometryRandomTools::nextLine3D;
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITester.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder, framelessTypeBuilder, methodFilter);
   }

   @Override
   @Test
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      signaturesToIgnore.add(new MethodSignature("set", Line3D.class));
      signaturesToIgnore.add(new MethodSignature("equals", Line3D.class));
      signaturesToIgnore.add(new MethodSignature("epsilonEquals", Line3D.class, Double.TYPE));
      signaturesToIgnore.add(new MethodSignature("geometricallyEquals", Line3D.class, Double.TYPE));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);

      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FrameLine3D.class, Line3D.class, true, 1, methodFilter);
   }

   @Test
   public void testSetMatchingFrame() throws Exception
   {
      Random random = new Random(544354);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

         FrameLine3DReadOnly source = EuclidFrameRandomTools.nextFrameLine3D(random, sourceFrame);
         FrameLine3D actual = EuclidFrameRandomTools.nextFrameLine3D(random, destinationFrame);

         actual.setMatchingFrame(source);

         FrameLine3D expected = new FrameLine3D(source);
         expected.changeFrame(destinationFrame);

         EuclidGeometryTestTools.assertLine3DEquals(expected, actual, EPSILON);
      }
   }
}
