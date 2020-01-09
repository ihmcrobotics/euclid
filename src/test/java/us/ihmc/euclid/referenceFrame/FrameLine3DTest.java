package us.ihmc.euclid.referenceFrame;

import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
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
      Random random = new Random(234235L);

      EuclidFrameAPITestTools.FrameTypeBuilder<? extends ReferenceFrameHolder> frameTypeBuilder = (frame, line) -> createFrameLine(frame,
                                                                                                                                   (Line3DReadOnly) line);
      EuclidFrameAPITestTools.GenericTypeBuilder framelessTypeBuilder = () -> EuclidGeometryRandomTools.nextLine3D(random);
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITestTools.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder, framelessTypeBuilder, methodFilter);
   }

   @Override
   @Test
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Map<String, Class<?>[]> framelessMethodsToIgnore = new HashMap<>();
      framelessMethodsToIgnore.put("set", new Class<?>[] {Line3D.class});
      framelessMethodsToIgnore.put("equals", new Class<?>[] {Line3D.class});
      framelessMethodsToIgnore.put("epsilonEquals", new Class<?>[] {Line3D.class, Double.TYPE});
      framelessMethodsToIgnore.put("geometricallyEquals", new Class<?>[] {Line3D.class, Double.TYPE});
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameLine3D.class, Line3D.class, true, 1, framelessMethodsToIgnore);
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
