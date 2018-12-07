package us.ihmc.euclid.tools;

import static us.ihmc.robotics.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;

public class TupleToolsTest
{
   @Test
   public void testEpsilonEqualsTuple2D() throws Exception
   {
      Random random = new Random(621541L);
      Tuple2DBasics tuple1 = new Point2D();
      Tuple2DBasics tuple2 = new Point2D();

      double epsilon = random.nextDouble();

      tuple1.setX(random.nextDouble());
      tuple1.setY(random.nextDouble());

      tuple2.setX(tuple1.getX() + 0.1 * epsilon);
      tuple2.setY(tuple1.getY() + 0.1 * epsilon);
      assertTrue(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 0.1 * epsilon);
      tuple2.setY(tuple1.getY() - 0.1 * epsilon);
      assertTrue(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 1.1 * epsilon);
      tuple2.setY(tuple1.getY() - 0.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 0.1 * epsilon);
      tuple2.setY(tuple1.getY() - 1.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + 1.1 * epsilon);
      tuple2.setY(tuple1.getY() + 0.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + 0.1 * epsilon);
      tuple2.setY(tuple1.getY() + 1.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + epsilon);
      tuple2.setY(tuple1.getY() + epsilon);
      assertTrue(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - epsilon);
      tuple2.setY(tuple1.getY() - epsilon);
      assertTrue(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));
   }

   @Test
   public void testEpsilonEqualsTuple3D() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3DBasics tuple1 = new Point3D();
      Tuple3DBasics tuple2 = new Point3D();

      double epsilon = random.nextDouble();

      tuple1.setX(random.nextDouble());
      tuple1.setY(random.nextDouble());
      tuple1.setZ(random.nextDouble());

      tuple2.setX(tuple1.getX() + 0.1 * epsilon);
      tuple2.setY(tuple1.getY() + 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() + 0.1 * epsilon);
      assertTrue(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 0.1 * epsilon);
      tuple2.setY(tuple1.getY() - 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() - 0.1 * epsilon);
      assertTrue(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 1.1 * epsilon);
      tuple2.setY(tuple1.getY() - 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() - 0.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 0.1 * epsilon);
      tuple2.setY(tuple1.getY() - 1.1 * epsilon);
      tuple2.setZ(tuple1.getZ() - 0.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 0.1 * epsilon);
      tuple2.setY(tuple1.getY() - 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() - 1.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + 1.1 * epsilon);
      tuple2.setY(tuple1.getY() + 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() + 0.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + 0.1 * epsilon);
      tuple2.setY(tuple1.getY() + 1.1 * epsilon);
      tuple2.setZ(tuple1.getZ() + 0.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + 0.1 * epsilon);
      tuple2.setY(tuple1.getY() + 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() + 1.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + epsilon);
      tuple2.setY(tuple1.getY() + epsilon);
      tuple2.setZ(tuple1.getZ() + epsilon);
      assertTrue(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - epsilon);
      tuple2.setY(tuple1.getY() - epsilon);
      tuple2.setZ(tuple1.getZ() - epsilon);
      assertTrue(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));
   }

   @Test
   public void testEpsilonEqualsTuple4D() throws Exception
   {
      Random random = new Random(621541L);
      Vector4DBasics tuple1 = new Vector4D();
      Vector4DBasics tuple2 = new Vector4D();

      double epsilon = random.nextDouble();

      tuple1.setX(random.nextDouble());
      tuple1.setY(random.nextDouble());
      tuple1.setZ(random.nextDouble());
      tuple1.setS(random.nextDouble());

      tuple2.setX(tuple1.getX() + 0.1 * epsilon);
      tuple2.setY(tuple1.getY() + 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() + 0.1 * epsilon);
      tuple2.setS(tuple1.getS() + 0.1 * epsilon);
      assertTrue(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 0.1 * epsilon);
      tuple2.setY(tuple1.getY() - 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() - 0.1 * epsilon);
      tuple2.setS(tuple1.getS() - 0.1 * epsilon);
      assertTrue(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 1.1 * epsilon);
      tuple2.setY(tuple1.getY() - 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() - 0.1 * epsilon);
      tuple2.setS(tuple1.getS() - 0.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 0.1 * epsilon);
      tuple2.setY(tuple1.getY() - 1.1 * epsilon);
      tuple2.setZ(tuple1.getZ() - 0.1 * epsilon);
      tuple2.setS(tuple1.getS() - 0.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 0.1 * epsilon);
      tuple2.setY(tuple1.getY() - 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() - 1.1 * epsilon);
      tuple2.setS(tuple1.getS() - 0.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 0.1 * epsilon);
      tuple2.setY(tuple1.getY() - 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() - 0.1 * epsilon);
      tuple2.setS(tuple1.getS() - 1.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + 1.1 * epsilon);
      tuple2.setY(tuple1.getY() + 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() + 0.1 * epsilon);
      tuple2.setS(tuple1.getS() + 0.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + 0.1 * epsilon);
      tuple2.setY(tuple1.getY() + 1.1 * epsilon);
      tuple2.setZ(tuple1.getZ() + 0.1 * epsilon);
      tuple2.setS(tuple1.getS() + 0.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + 0.1 * epsilon);
      tuple2.setY(tuple1.getY() + 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() + 1.1 * epsilon);
      tuple2.setS(tuple1.getS() + 0.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + 0.1 * epsilon);
      tuple2.setY(tuple1.getY() + 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() + 0.1 * epsilon);
      tuple2.setS(tuple1.getS() + 1.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + epsilon);
      tuple2.setY(tuple1.getY() + epsilon);
      tuple2.setZ(tuple1.getZ() + epsilon);
      tuple2.setS(tuple1.getZ() + epsilon);
      assertTrue(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - epsilon);
      tuple2.setY(tuple1.getY() - epsilon);
      tuple2.setZ(tuple1.getZ() - epsilon);
      tuple2.setS(tuple1.getS() - epsilon);
      assertTrue(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));
   }
}
