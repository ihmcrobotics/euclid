package us.ihmc.euclid.geometry.tools;

import static us.ihmc.euclid.tools.EuclidCoreRandomTools.generateRandomPoint3D;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.generateRandomVector3D;

import java.util.Random;

import us.ihmc.euclid.geometry.Line3D;

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
    * @throws RuntimeException if {@code minMax < 0}.
    */
   public static Line3D generateRandomLine3D(Random random, double pointMinMax)
   {
      return new Line3D(generateRandomPoint3D(random, pointMinMax), generateRandomVector3D(random));
   }
}
