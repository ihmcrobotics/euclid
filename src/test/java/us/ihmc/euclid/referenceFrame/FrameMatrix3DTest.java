package us.ihmc.euclid.referenceFrame;

import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.FrameTypeCopier;
import us.ihmc.euclid.referenceFrame.api.RandomFramelessTypeBuilder;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameMatrix3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameMatrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;

public class FrameMatrix3DTest extends FrameMatrix3DReadOnlyTest<FrameMatrix3D>
{
   public static final double EPSILON = 1.0e-15;

   @Override
   public FrameMatrix3D createFrameMatrix3D(ReferenceFrame referenceFrame, Matrix3DReadOnly pose)
   {
      return new FrameMatrix3D(referenceFrame, pose);
   }

   @Test
   public void testConsistencyWithMatrix3D()
   {
      FrameTypeCopier frameTypeBuilder = (frame, matrix) -> createFrameMatrix3D(frame, (Matrix3DReadOnly) matrix);
      RandomFramelessTypeBuilder framelessTypeBuilder = EuclidCoreRandomTools::nextMatrix3D;
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITester.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder, framelessTypeBuilder, methodFilter);
   }

   @Override
   @Test
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Map<String, Class<?>[]> framelessMethodsToIgnore = new HashMap<>();
      framelessMethodsToIgnore.put("set", new Class<?>[] {Matrix3D.class});
      framelessMethodsToIgnore.put("equals", new Class<?>[] {Matrix3D.class});
      framelessMethodsToIgnore.put("epsilonEquals", new Class<?>[] {Matrix3D.class, Double.TYPE});
      framelessMethodsToIgnore.put("geometricallyEquals", new Class<?>[] {Matrix3D.class, Double.TYPE});
      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FrameMatrix3D.class, Matrix3D.class, false, 1, framelessMethodsToIgnore);
   }

   @Test
   public void testSetMatchingFrame() throws Exception
   {
      Random random = new Random(544354);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);

         FrameMatrix3DReadOnly source = EuclidFrameRandomTools.nextFrameMatrix3D(random, sourceFrame);
         FixedFrameMatrix3DBasics actual = EuclidFrameRandomTools.nextFrameMatrix3D(random, destinationFrame);

         actual.setMatchingFrame(source);

         FrameMatrix3D expected = new FrameMatrix3D(source);
         expected.changeFrame(destinationFrame);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPSILON);
      }
   }
}
