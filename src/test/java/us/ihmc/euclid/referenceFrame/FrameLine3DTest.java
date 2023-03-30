package us.ihmc.euclid.referenceFrame;

import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPIDefaultConfiguration;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.FrameTypeCopier;
import us.ihmc.euclid.referenceFrame.api.MethodSignature;
import us.ihmc.euclid.referenceFrame.api.RandomFramelessTypeBuilder;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;

public class FrameLine3DTest extends FrameLine3DReadOnlyTest<FrameLine3D>
{
   public static final double EPSILON = 1.0e-15;

   @BeforeEach
   public void disableNameRestriction()
   {
      ReferenceFrame.getWorldFrame().setNameRestrictionLevel(FrameNameRestrictionLevel.NONE);
   }

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
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && !m.getName().equals("epsilonEquals") && !m.getName().equals("toString");
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder,
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
      signaturesToIgnore.add(new MethodSignature("set", Line3D.class));
      signaturesToIgnore.add(new MethodSignature("equals", Line3D.class));
      signaturesToIgnore.add(new MethodSignature("epsilonEquals", Line3D.class, Double.TYPE));
      signaturesToIgnore.add(new MethodSignature("geometricallyEquals", Line3D.class, Double.TYPE));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);

      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertOverloadingWithFrameObjects(FrameLine3D.class, Line3D.class, true, 1, methodFilter);
   }

   @Test
   public void testSetMatchingFrame() throws Exception
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertSetMatchingFramePreserveFunctionality(EuclidFrameRandomTools::nextFrameLine3D, EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);

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

         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testSetIncludingFrame()
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertSetIncludingFramePreserveFunctionality(EuclidFrameRandomTools::nextFrameLine2D, EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }
}
