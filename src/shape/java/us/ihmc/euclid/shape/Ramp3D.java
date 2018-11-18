package us.ihmc.euclid.shape;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

/**
 * {@code Ramp3D} represents a 3D shape with a triangular section in the XZ-plane.
 * <p>
 * Shape description:
 * <ul>
 * <li>The slope face starts from {@code x=0.0}, {@code z=0.0} to end at {@code x=size.getX()},
 * {@code z=size.getZ()}.
 * <li>The bottom face is horizontal (XY-plane) at {@code z=0.0}.
 * <li>The rear face is vertical (YZ-plane) at {@code x=size.getX()}.
 * <li>The left face is vertical (XZ-plane) at {@code y=-size.getY()/2.0}.
 * <li>The right face is vertical (XZ-plane) at {@code y=size.getY()/2.0}.
 * </ul>
 * </p>
 */
public class Ramp3D extends Shape3D implements GeometryObject<Ramp3D>
{
   /** Size of this ramp's bounding box. */
   private final Vector3D size = new Vector3D()
   {
      @Override
      public void setX(double x)
      {
         if (x < 0.0)
            throw new IllegalArgumentException("The x-size of a Ramp3D cannot be negative: " + x);
         super.setX(x);
         updateRamp();
      }

      @Override
      public void setY(double y)
      {
         if (y < 0.0)
            throw new IllegalArgumentException("The y-size of a Ramp3D cannot be negative: " + y);
         super.setY(y);
      }

      @Override
      public void setZ(double z)
      {
         if (z < 0.0)
            throw new IllegalArgumentException("The z-size of a Ramp3D cannot be negative: " + z);
         super.setZ(z);
         updateRamp();
      }
   };

   /** Length of the slope face of this ramp. */
   private double rampLength;
   /**
    * Positive angle in [0, <i>pi</i>] representing the angle formed by the bottom face and the slope
    * face.
    */
   private double angleOfRampIncline;

   /**
    * Creates a new ramp 3D and initializes its length, width, and height to {@code 1.0}.
    */
   public Ramp3D()
   {
      this(1.0, 1.0, 1.0);
   }

   /**
    * Creates a new ramp 3D identical to {@code other}.
    *
    * @param other the other ramp to copy. Not modified.
    */
   public Ramp3D(Ramp3D other)
   {
      set(other);
   }

   /**
    * Creates a new ramp 3D and initializes its size.
    *
    * @param length the size of this ramp along the x-axis.
    * @param width the size of this ramp along the y-axis.
    * @param height the size of this ramp along the z-axis.
    * @throws IllegalArgumentException if any of {@code length}, {@code width}, or {@code height} is
    *            negative.
    */
   public Ramp3D(double length, double width, double height)
   {
      setSize(length, width, height);
   }

   /**
    * Creates a new ramp 3D and initializes its pose and size.
    *
    * @param pose the position and orientation for this ramp. Not modified.
    * @param length the size of this ramp along the x-axis.
    * @param width the size of this ramp along the y-axis.
    * @param height the size of this ramp along the z-axis.
    * @throws IllegalArgumentException if any of {@code length}, {@code width}, or {@code height} is
    *            negative.
    */
   public Ramp3D(RigidBodyTransformReadOnly pose, double length, double width, double height)
   {
      setPose(pose);
      setSize(length, width, height);
   }

   /**
    * Creates a new ramp 3D and initializes its pose and size.
    *
    * @param pose the position and orientation for this ramp. Not modified.
    * @param length the size of this ramp along the x-axis.
    * @param width the size of this ramp along the y-axis.
    * @param height the size of this ramp along the z-axis.
    * @throws IllegalArgumentException if any of {@code length}, {@code width}, or {@code height} is
    *            negative.
    */
   public Ramp3D(Pose3DReadOnly pose, double length, double width, double height)
   {
      setPose(pose);
      setSize(length, width, height);
   }

   /**
    * Copies the {@code other} ramp data into {@code this}.
    *
    * @param other the other ramp to copy. Not modified.
    */
   @Override
   public void set(Ramp3D other)
   {
      setPose(other);
      setSize(other.size);
   }

   /**
    * Sets the size along the x-axis for this ramp.
    *
    * @param sizeX the size of this ramp along the x-axis.
    * @throws IllegalArgumentException if {@code length} is negative.
    */
   public void setSizeX(double sizeX)
   {
      size.setX(sizeX);
   }

   /**
    * Sets the size along the y-axis for this ramp.
    *
    * @param sizeY the size of this ramp along the y-axis.
    * @throws IllegalArgumentException if {@code width} is negative.
    */
   public void setSizeY(double sizeY)
   {
      size.setY(sizeY);
   }

   /**
    * Sets the size along the z-axis for this ramp.
    *
    * @param sizeZ the size of this ramp along the z-axis.
    * @throws IllegalArgumentException if {@code height} is negative.
    */
   public void setSizeZ(double sizeZ)
   {
      size.setZ(sizeZ);
   }

   /**
    * Sets the size of this ramp.
    *
    * @param size tuple with the new size for this ramp. Not modified.
    * @throws IllegalArgumentException if any of {@code size} components is negative.
    */
   public void setSize(Tuple3DReadOnly size)
   {
      setSize(size.getX(), size.getY(), size.getZ());
   }

   /**
    * Sets the size of this ramp.
    *
    * @param sizeX the size of this ramp along the x-axis.
    * @param sizeY the size of this ramp along the y-axis.
    * @param sizeZ the size of this ramp along the z-axis.
    * @throws IllegalArgumentException if any of {@code length}, {@code width}, or {@code height} is
    *            negative.
    */
   public void setSize(double sizeX, double sizeY, double sizeZ)
   {
      size.set(sizeX, sizeY, sizeZ);
   }

   private void updateRamp()
   {
      rampLength = Math.sqrt(EuclidCoreTools.normSquared(size.getX(), size.getZ()));
      angleOfRampIncline = Math.atan(size.getZ() / size.getX());
   }

   /**
    * Gets the size of this ramp along the x-axis.
    *
    * @return this ramp's length.
    */
   public double getSizeX()
   {
      return size.getX();
   }

   /**
    * Gets the size of this ramp along the y-axis.
    *
    * @return this ramp's width.
    */
   public double getSizeY()
   {
      return size.getY();
   }

   /**
    * Gets the size of this ramp along the z-axis.
    *
    * @return this ramp's height.
    */
   public double getSizeZ()
   {
      return size.getZ();
   }

   /**
    * Gets the length of this ramp's slope part.
    * <p>
    * Note that this is different than {@link #getSizeX()}. The returned value is equal to:
    * &radic;(this.length<sup>2</sup> + this.height<sup>2</sup>)
    * </p>
    *
    * @return the length of the slope.
    */
   public double getRampLength()
   {
      return rampLength;
   }

   /**
    * Computes and packs the surface normal of the slope face of this ramp.
    *
    * @param surfaceNormalToPack the surface normal of the slope. Modified.
    */
   public void getRampSurfaceNormal(Vector3DBasics surfaceNormalToPack)
   {
      surfaceNormalToPack.set(-size.getZ() / rampLength, 0.0, size.getX() / rampLength);
      transformToWorld(surfaceNormalToPack);
   }

   /**
    * Gets the angle formed by the slope and the bottom face.
    * <p>
    * The angle is positive and in [0, <i>pi</i>].
    * </p>
    *
    * @return the slope angle.
    */
   public double getRampIncline()
   {
      return angleOfRampIncline;
   }

   /** {@inheritDoc} */
   @Override
   protected double evaluateQuery(Point3DReadOnly query, Point3DBasics closestPointToPack, Vector3DBasics normalToPack)
   {
      double rampDirectionX = size.getX() / rampLength;
      double rampDirectionZ = size.getZ() / rampLength;
      double rampNormalX = -rampDirectionZ;
      double rampNormalZ = rampDirectionX;

      double x = query.getX();
      double y = query.getY();
      double z = query.getZ();

      double halfWidth = 0.5 * size.getY();
      if (z < 0.0)
      { // Query is below the ramp
         double xClosest = EuclidCoreTools.clamp(x, 0.0, size.getX());
         double yClosest = EuclidCoreTools.clamp(y, -0.5 * size.getY(), halfWidth);
         double zClosest = 0.0;

         if (closestPointToPack != null)
         {
            closestPointToPack.set(xClosest, yClosest, zClosest);
         }

         if (xClosest == x && yClosest == y)
         {
            if (normalToPack != null)
            {
               normalToPack.set(0.0, 0.0, -1.0);
            }

            return -z;
         }
         else
         {
            return computeNormalAndDistanceFromClosestPoint(x, y, z, xClosest, yClosest, zClosest, normalToPack);
         }
      }
      else if (x > size.getX() || EuclidGeometryTools.isPoint2DOnSideOfLine2D(x, z, size.getX(), size.getZ(), rampNormalX, rampNormalZ, false))
      { // Query is beyond the ramp
         double xClosest = size.getX();
         double yClosest = EuclidCoreTools.clamp(y, -0.5 * size.getY(), halfWidth);
         double zClosest = EuclidCoreTools.clamp(z, 0.0, size.getZ());

         if (closestPointToPack != null)
         {
            closestPointToPack.set(xClosest, yClosest, zClosest);
         }

         if (yClosest == y && zClosest == z)
         {
            if (normalToPack != null)
            {
               normalToPack.set(1.0, 0.0, 0.0);
            }

            return x - size.getX();
         }
         else
         {
            return computeNormalAndDistanceFromClosestPoint(x, y, z, xClosest, yClosest, zClosest, normalToPack);
         }
      }
      else if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(x, z, 0.0, 0.0, rampNormalX, rampNormalZ, true))
      { // Query is before ramp and the closest point lies on starting edge
         double xClosest = 0.0;
         double yClosest = EuclidCoreTools.clamp(y, -0.5 * size.getY(), halfWidth);
         double zClosest = 0.0;

         if (closestPointToPack != null)
         {
            closestPointToPack.set(xClosest, yClosest, zClosest);
         }

         return computeNormalAndDistanceFromClosestPoint(x, y, z, xClosest, yClosest, zClosest, normalToPack);
      }
      else if (Math.abs(y) > halfWidth)
      { // Query is on either side of the ramp
         double yClosest = Math.copySign(halfWidth, y);

         if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(x, z, 0.0, 0.0, rampDirectionX, rampDirectionZ, false))
         { // Query is below the slope
            double xClosest = x;
            double zClosest = z;

            if (closestPointToPack != null)
            {
               closestPointToPack.set(xClosest, yClosest, zClosest);
            }

            if (normalToPack != null)
            {
               normalToPack.set(0.0, Math.copySign(1.0, y), 0.0);
            }

            return Math.abs(y) - halfWidth;
         }
         else
         { // Query is above the slope
            double dot = x * rampDirectionX + z * rampDirectionZ;
            double xClosest = dot * rampDirectionX;
            double zClosest = dot * rampDirectionZ;

            if (closestPointToPack != null)
            {
               closestPointToPack.set(xClosest, yClosest, zClosest);
            }

            return computeNormalAndDistanceFromClosestPoint(x, y, z, xClosest, yClosest, zClosest, normalToPack);
         }
      }
      else if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(x, z, 0.0, 0.0, rampDirectionX, rampDirectionZ, true))
      { // Query is directly above the slope part
         double dot = x * rampDirectionX + z * rampDirectionZ;
         double xClosest = dot * rampDirectionX;
         double yClosest = y;
         double zClosest = dot * rampDirectionZ;

         if (closestPointToPack != null)
         {
            closestPointToPack.set(xClosest, yClosest, zClosest);
         }

         if (normalToPack != null)
         {
            normalToPack.set(rampNormalX, 0.0, rampNormalZ);
         }

         return rampDirectionX * z - x * rampDirectionZ;
      }
      else
      { // Query is inside the ramp
         double distanceToRightFace = -(-halfWidth - y);
         double distanceToLeftFace = halfWidth - y;
         double distanceToRearFace = size.getX() - x;
         double distanceToBottomFace = z;
         double distanceToSlopeFace = -(rampDirectionX * z - x * rampDirectionZ);

         if (isFirstValueMinimum(distanceToRightFace, distanceToLeftFace, distanceToRearFace, distanceToBottomFace, distanceToSlopeFace))
         { // Query is closer to the right face
            if (closestPointToPack != null)
            {
               closestPointToPack.set(x, -halfWidth, z);
            }

            if (normalToPack != null)
            {
               normalToPack.set(0.0, -1.0, 0.0);
            }

            return -distanceToRightFace;
         }
         else if (isFirstValueMinimum(distanceToLeftFace, distanceToRearFace, distanceToBottomFace, distanceToSlopeFace))
         { // Query is closer to the left face
            if (closestPointToPack != null)
            {
               closestPointToPack.set(x, halfWidth, z);
            }

            if (normalToPack != null)
            {
               normalToPack.set(0.0, 1.0, 0.0);
            }

            return -distanceToLeftFace;
         }
         else if (isFirstValueMinimum(distanceToRearFace, distanceToBottomFace, distanceToSlopeFace))
         { // Query is closer to the rear face
            if (closestPointToPack != null)
            {
               closestPointToPack.set(size.getX(), y, z);
            }

            if (normalToPack != null)
            {
               normalToPack.set(1.0, 0.0, 0.0);
            }

            return -distanceToRearFace;
         }
         else if (distanceToBottomFace <= distanceToSlopeFace)
         { // Query is closer to the bottom face
            if (closestPointToPack != null)
            {
               closestPointToPack.set(x, y, 0.0);
            }

            if (normalToPack != null)
            {
               normalToPack.set(0.0, 0.0, -1.0);
            }

            return -distanceToBottomFace;
         }
         else
         { // Query is closer to the slope face
            if (closestPointToPack != null)
            {
               double dot = x * rampDirectionX + z * rampDirectionZ;
               double xClosest = dot * rampDirectionX;
               double yClosest = y;
               double zClosest = dot * rampDirectionZ;

               closestPointToPack.set(xClosest, yClosest, zClosest);
            }

            if (normalToPack != null)
            {
               normalToPack.set(rampNormalX, 0.0, rampNormalZ);
            }

            return -distanceToSlopeFace;
         }
      }
   }

   private static double computeNormalAndDistanceFromClosestPoint(double x, double y, double z, double xClosest, double yClosest, double zClosest,
                                                                  Vector3DBasics normalToPack)
   {
      double dx = x - xClosest;
      double dy = y - yClosest;
      double dz = z - zClosest;

      double distance = Math.sqrt(EuclidCoreTools.normSquared(dx, dy, dz));

      if (normalToPack != null)
      {
         normalToPack.set(dx, dy, dz);
         normalToPack.scale(1.0 / distance);
      }

      return distance;
   }

   private static boolean isFirstValueMinimum(double possibleMin, double value1, double value2, double value3, double value4)
   {
      return possibleMin <= value1 && possibleMin <= value2 && possibleMin <= value3 && possibleMin <= value4;
   }

   private static boolean isFirstValueMinimum(double possibleMin, double value1, double value2, double value3)
   {
      return possibleMin <= value1 && possibleMin <= value2 && possibleMin <= value3;
   }

   private static boolean isFirstValueMinimum(double possibleMin, double value1, double value2)
   {
      return possibleMin <= value1 && possibleMin <= value2;
   }

   /** {@inheritDoc} */
   @Override
   protected boolean isInsideEpsilonShapeFrame(Point3DReadOnly query, double epsilon)
   {
      if (query.getZ() < -epsilon)
         return false;

      if (query.getX() > size.getX() + epsilon)
         return false;

      double halfWidth = 0.5 * size.getY() + epsilon;

      if (query.getY() < -halfWidth || query.getY() > halfWidth)
         return false;

      double rampDirectionX = size.getX() / rampLength;
      double rampDirectionZ = size.getZ() / rampLength;

      // Computing the signed distance between the query and the slope face, negative value means the query is below the slope.
      if (rampDirectionX * query.getZ() - query.getX() * rampDirectionZ > epsilon)
         return false;

      return true;
   }

   /**
    * Tests separately and on a per component basis if the pose and the size of this ramp and
    * {@code other}'s pose and size are equal to an {@code epsilon}.
    *
    * @param other the other ramp which pose and size is to be compared against this ramp pose and
    *           size. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two ramps are equal component-wise, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Ramp3D other, double epsilon)
   {
      return size.epsilonEquals(other.size, epsilon) && super.epsilonEqualsPose(other, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   public void setToZero()
   {
      super.setToZero();
      size.setToZero();
   }

   /** {@inheritDoc} */
   @Override
   public void setToNaN()
   {
      super.setToNaN();
      size.setToNaN();
   }

   /** {@inheritDoc} */
   @Override
   public boolean containsNaN()
   {
      return super.containsNaN() || size.containsNaN();
   }

   /**
    * Provides a {@code String} representation of this ramp 3D as follows:<br>
    * Ramp 3D: size = (length, width, height), pose = <br>
    * m00, m01, m02 | m03 <br>
    * m10, m11, m12 | m13 <br>
    * m20, m21, m22 | m23
    *
    * @return the {@code String} representing this box 3D.
    */
   @Override
   public String toString()
   {
      return "Ramp 3D: size = " + size + ", + pose =\n" + getPoseString();
   }

   /**
    * Compares {@code this} and {@code other} to determine if the two ramps are geometrically similar,
    * i.e. the difference between their size are less than or equal to {@code epsilon} and their poses
    * are geometrically similar given {@code epsilon}.
    *
    * @param other the ramp to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the ramps represent the same geometry, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(Ramp3D other, double epsilon)
   {
      return shapePose.geometricallyEquals(other.shapePose, epsilon) && size.epsilonEquals(other.size, epsilon);
   }
}
