package us.ihmc.euclid.shape;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.shape.interfaces.Sphere3DBasics;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

/**
 * {@code Sphere3D} represents a 3D sphere defined by its radius and with its origin at its center.
 */
public class Sphere3D extends Shape3D implements Sphere3DBasics, GeometryObject<Sphere3D>
{
   /** The radius of this sphere. */
   private double radius;

   /**
    * Creates a new sphere 3D with a radius of {@code 1}.
    */
   public Sphere3D()
   {
      this(1.0);
   }

   /**
    * Creates a new sphere 3D identical to {@code other}
    *
    * @param other the other sphere to copy. Not modified.
    */
   public Sphere3D(Sphere3D other)
   {
      set(other);
   }

   /**
    * Creates a new sphere 3D and initializes its radius.
    *
    * @param radius the radius for this sphere.
    * @throws IllegalArgumentException if {@code radius} is negative.
    */
   public Sphere3D(double radius)
   {
      setRadius(radius);
   }

   /**
    * Creates a new sphere 3D and initializes its position and radius.
    *
    * @param center the coordinates of this sphere. Not modified.
    * @param radius the radius for this sphere.
    * @throws IllegalArgumentException if {@code radius} is negative.
    */
   public Sphere3D(Point3DReadOnly center, double radius)
   {
      set(center, radius);
   }

   /**
    * Creates a new sphere 3D and initializes its position and radius.
    *
    * @param centerX the x-coordinate of this sphere. Not modified.
    * @param centerY the y-coordinate of this sphere. Not modified.
    * @param centerZ the z-coordinate of this sphere. Not modified.
    * @param radius the radius for this sphere.
    * @throws IllegalArgumentException if {@code radius} is negative.
    */
   public Sphere3D(double centerX, double centerY, double centerZ, double radius)
   {
      set(centerX, centerY, centerZ, radius);
   }

   /**
    * Gets the radius of this sphere.
    *
    * @return the value of the radius.
    */
   @Override
   public double getRadius()
   {
      return radius;
   }

   /**
    * Sets the radius of this sphere.
    *
    * @param radius the radius for this sphere.
    * @throws IllegalArgumentException if {@code radius} is negative.
    */
   @Override
   public void setRadius(double radius)
   {
      if (radius < 0.0)
         throw new IllegalArgumentException("The radius of a Sphere 3D cannot be negative.");
      this.radius = radius;
   }

   /**
    * Copies the {@code other} sphere data into {@code this}.
    *
    * @param other the other sphere to copy. Not modified.
    */
   @Override
   public void set(Sphere3D other)
   {
      Sphere3DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   protected boolean isInsideEpsilonShapeFrame(Point3DReadOnly query, double epsilon)
   {
      return EuclidShapeTools.isPoint3DInsideSphere3D(query, getRadius(), epsilon);
   }

   /** {@inheritDoc} */
   @Override
   protected double evaluateQuery(Point3DReadOnly query, Point3DBasics closestPointToPack, Vector3DBasics normalToPack)
   {
      return EuclidShapeTools.evaluatePoint3DWithSphere3D(query, closestPointToPack, normalToPack, getRadius());
   }

   /**
    * Tests separately and on a per component basis if the pose and the radius of this sphere and
    * {@code other}'s pose and radius are equal to an {@code epsilon}.
    *
    * @param other the other sphere which pose and radius is to be compared against this radius pose
    *           and radius. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two spheres are equal component-wise, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Sphere3D other, double epsilon)
   {
      return Sphere3DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two spheres are geometrically similar,
    * i.e. the position of each sphere is geometrically similar given {@code epsilon} and the
    * difference between the radius of each sphere is less than or equal to {@code epsilon}.
    *
    * @param other the sphere to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two boxes represent the same geometry, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(Sphere3D other, double epsilon)
   {
      return Sphere3DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Provides a {@code String} representation of this sphere 3D as follows:<br>
    * Sphere 3D: radius = r, pose = <br>
    * m00, m01, m02 | m03 <br>
    * m10, m11, m12 | m13 <br>
    * m20, m21, m22 | m23
    *
    * @return the {@code String} representing this box 3D.
    */
   @Override
   public String toString()
   {
      return "Sphere 3D: radius = " + radius + ", pose=\n" + getPoseString();
   }
}
