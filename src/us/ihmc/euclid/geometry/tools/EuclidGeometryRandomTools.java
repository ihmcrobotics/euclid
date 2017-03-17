package us.ihmc.euclid.geometry.tools;

import static us.ihmc.euclid.tools.EuclidCoreRandomTools.generateRandomPoint2D;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.generateRandomPoint3D;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.generateRandomVector2D;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.generateRandomVector3D;

import java.util.Random;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public abstract class EuclidGeometryRandomTools
{
   /**
    * Generates a random line 3D.
    * <p>
    * <ul>
    * <li>{@code point}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code direction}<sub>i</sub> &in; [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random line 3D.
    */
   public static Line3D generateRandomLine3D(Random random)
   {
      return new Line3D(generateRandomPoint3D(random), generateRandomVector3D(random));
   }

   /**
    * Generates a random line 3D.
    * <p>
    * <ul>
    * <li>{@code point}<sub>i</sub> &in; [-pointMinMax; pointMinMax].
    * <li>{@code direction}<sub>i</sub> &in; [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @param pointMinMax the maximum absolute value for each coordinate of the line's point.
    * @return the random line 3D.
    * @throws RuntimeException if {@code pointMinMax < 0}.
    */
   public static Line3D generateRandomLine3D(Random random, double pointMinMax)
   {
      return new Line3D(generateRandomPoint3D(random, pointMinMax), generateRandomVector3D(random));
   }

   /**
    * Generates a random line segment 3D.
    * <p>
    * <ul>
    * <li>{@code firstEndpoint}<sub>i</sub> &in; [-1.0; 1.0].
    * <li>{@code secondEndpoint}<sub>i</sub> &in; [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random line segment 3D.
    */
   public static LineSegment3D generateRandomLineSegment3D(Random random)
   {
      return new LineSegment3D(generateRandomPoint3D(random), generateRandomPoint3D(random));
   }

   /**
    * Generates a random line segment 3D.
    * <p>
    * <ul>
    * <li>{@code firstEndpoint}<sub>i</sub> &in; [-minMax; minMax].
    * <li>{@code secondEndpoint}<sub>i</sub> &in; [-minMax; minMax].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @param minMax the maximum absolute value for each coordinate of the line segment's endpoints.
    * @return the random line segment 3D.
    * @throws RuntimeException if {@code minMax < 0}.
    */
   public static LineSegment3D generateRandomLineSegment3D(Random random, double minMax)
   {
      return new LineSegment3D(generateRandomPoint3D(random, minMax), generateRandomPoint3D(random, minMax));
   }

   /**
    * Generates a random bounding box from random center location and random size.
    * 
    * @param random the random generator to use.
    * @param centerMinMax the maximum absolute value for each coordinate of the bounding box center.
    * @param sizeMax the maximum size along each axis for the bounding box.
    * @return the random bounding box.
    * @throws RuntimeException if {@code centerMinMax < 0} or {@code sizeMax < 0}.
    */
   public static BoundingBox2D generateRandomBoundingBox2D(Random random, double centerMinMax, double sizeMax)
   {
      Point2D center = generateRandomPoint2D(random, centerMinMax);
      Vector2D halfSize = generateRandomVector2D(random, 0.0, sizeMax / 2.0);
      return BoundingBox2D.createUsingCenterAndPlusMinusVector(center, halfSize);
   }
   
   /**
    * Generates a random bounding box from random center location and random size.
    * 
    * @param random the random generator to use.
    * @param centerMinMax the maximum absolute value for each coordinate of the bounding box center.
    * @param sizeMax the maximum size along each axis for the bounding box.
    * @return the random bounding box.
    * @throws RuntimeException if {@code centerMinMax < 0} or {@code sizeMax < 0}.
    */
   public static BoundingBox3D generateRandomBoundingBox3D(Random random, double centerMinMax, double sizeMax)
   {
      Point3D center = generateRandomPoint3D(random, centerMinMax);
      Vector3D halfSize = generateRandomVector3D(random, 0.0, sizeMax / 2.0);
      return BoundingBox3D.createUsingCenterAndPlusMinusVector(center, halfSize);
   }
}
