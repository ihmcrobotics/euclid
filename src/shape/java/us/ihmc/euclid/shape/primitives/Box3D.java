package us.ihmc.euclid.shape.primitives;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.IntermediateVariableSupplier;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DChangeListener;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.tools.EuclidCoreFactories;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Implementation of a box 3D.
 * <p>
 * A box 3D is represented by its size, the position of its center, and its orientation.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class Box3D implements Box3DBasics, GeometryObject<Box3D>
{
   private final List<Shape3DChangeListener> changeListeners = new ArrayList<>();
   /** Pose of this box. */
   private final Shape3DPose pose = new Shape3DPose();
   /** Current supplier to use for storing intermediate results. */
   private IntermediateVariableSupplier supplier = IntermediateVariableSupplier.defaultIntermediateVariableSupplier();

   /**
    * Represents the sizeX, sizeY, and sizeZ of this box.
    */
   private final Vector3DBasics size = EuclidCoreFactories.newObservableVector3DBasics((axis, newValue) ->
   {
      checkSizePositive(axis);
      notifyChangeListeners();
   }, null, new Vector3D(1.0, 1.0, 1.0));

   private final Vector3DReadOnly halfSize = EuclidCoreFactories.newLinkedVector3DReadOnly(() -> 0.5, size);

   /**
    * Creates a 1-by-1-by-1 box 3D.
    */
   public Box3D()
   {
      Box3DVertex v0 = new Box3DVertex(pose, () -> +halfSize.getX(), () -> +halfSize.getY(), () -> +halfSize.getZ());
      Box3DVertex v1 = new Box3DVertex(pose, () -> +halfSize.getX(), () -> -halfSize.getY(), () -> +halfSize.getZ());
      Box3DVertex v2 = new Box3DVertex(pose, () -> -halfSize.getX(), () -> -halfSize.getY(), () -> +halfSize.getZ());
      Box3DVertex v3 = new Box3DVertex(pose, () -> -halfSize.getX(), () -> +halfSize.getY(), () -> +halfSize.getZ());
      Box3DVertex v4 = new Box3DVertex(pose, () -> +halfSize.getX(), () -> +halfSize.getY(), () -> -halfSize.getZ());
      Box3DVertex v5 = new Box3DVertex(pose, () -> +halfSize.getX(), () -> -halfSize.getY(), () -> -halfSize.getZ());
      Box3DVertex v6 = new Box3DVertex(pose, () -> -halfSize.getX(), () -> -halfSize.getY(), () -> -halfSize.getZ());
      Box3DVertex v7 = new Box3DVertex(pose, () -> -halfSize.getX(), () -> +halfSize.getY(), () -> -halfSize.getZ());
      changeListeners.addAll(Arrays.asList(v0, v1, v2, v3, v4, v5, v6, v7));

      // Face x+
      Box3DHalfEdge xPlusE0 = new Box3DHalfEdge(v0, v4);
      Box3DHalfEdge xPlusE1 = new Box3DHalfEdge(v4, v5);
      Box3DHalfEdge xPlusE2 = new Box3DHalfEdge(v5, v1);
      Box3DHalfEdge xPlusE3 = new Box3DHalfEdge(v1, v0);
      // Edges of face x-
      Box3DHalfEdge xMinusE0 = new Box3DHalfEdge(v3, v2);
      Box3DHalfEdge xMinusE1 = new Box3DHalfEdge(v2, v6);
      Box3DHalfEdge xMinusE2 = new Box3DHalfEdge(v6, v7);
      Box3DHalfEdge xMinusE3 = new Box3DHalfEdge(v7, v3);
      // Edges of face y+
      Box3DHalfEdge yPlusE0 = new Box3DHalfEdge(v0, v3);
      Box3DHalfEdge yPlusE1 = new Box3DHalfEdge(v3, v7);
      Box3DHalfEdge yPlusE2 = new Box3DHalfEdge(v7, v4);
      Box3DHalfEdge yPlusE3 = new Box3DHalfEdge(v4, v0);
      // Edges of face y-
      Box3DHalfEdge yMinusE0 = new Box3DHalfEdge(v1, v5);
      Box3DHalfEdge yMinusE1 = new Box3DHalfEdge(v5, v6);
      Box3DHalfEdge yMinusE2 = new Box3DHalfEdge(v6, v2);
      Box3DHalfEdge yMinusE3 = new Box3DHalfEdge(v2, v1);
      // Edges of face z+
      Box3DHalfEdge zPlusE0 = new Box3DHalfEdge(v0, v1);
      Box3DHalfEdge zPlusE1 = new Box3DHalfEdge(v1, v2);
      Box3DHalfEdge zPlusE2 = new Box3DHalfEdge(v2, v3);
      Box3DHalfEdge zPlusE3 = new Box3DHalfEdge(v3, v0);
      // Edges of face z-
      Box3DHalfEdge zMinusE0 = new Box3DHalfEdge(v4, v7);
      Box3DHalfEdge zMinusE1 = new Box3DHalfEdge(v7, v6);
      Box3DHalfEdge zMinusE2 = new Box3DHalfEdge(v6, v5);
      Box3DHalfEdge zMinusE3 = new Box3DHalfEdge(v5, v4);

      Box3DFace xPlusFace = new Box3DFace(pose, Axis3D.X, xPlusE0, xPlusE1, xPlusE2, xPlusE3);
      Box3DFace xMinusFace = new Box3DFace(pose, Axis3D.X.negated(), xMinusE0, xMinusE1, xMinusE2, xMinusE3);
      Box3DFace yPlusFace = new Box3DFace(pose, Axis3D.Y, yPlusE0, yPlusE1, yPlusE2, yPlusE3);
      Box3DFace yMinusFace = new Box3DFace(pose, Axis3D.Y.negated(), yMinusE0, yMinusE1, yMinusE2, yMinusE3);
      Box3DFace zPlusFace = new Box3DFace(pose, Axis3D.Z, zPlusE0, zPlusE1, zPlusE2, zPlusE3);
      Box3DFace zMinusFace = new Box3DFace(pose, Axis3D.Z.negated(), zMinusE0, zMinusE1, zMinusE2, zMinusE3);
      changeListeners.addAll(Arrays.asList(xPlusFace, xMinusFace, yPlusFace, yMinusFace, zPlusFace, zMinusFace));

      pose.addChangeListeners(changeListeners);
   }

   /**
    * Creates a new box 3D and initializes its size.
    *
    * @param sizeX the size of this box along the x-axis.
    * @param sizeY the size of this box along the y-axis.
    * @param sizeZ the size of this box along the z-axis.
    * @throws IllegalArgumentException if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ} is
    *                                  negative.
    */
   public Box3D(double sizeX, double sizeY, double sizeZ)
   {
      this();
      getSize().set(sizeX, sizeY, sizeZ);
   }

   /**
    * Creates a new box 3D and initializes its pose and size.
    *
    * @param position    the position of this box. Not modified.
    * @param orientation the orientation of this box. Not modified.
    * @param sizeX       the size of this box along the x-axis.
    * @param sizeY       the size of this box along the y-axis.
    * @param sizeZ       the size of this box along the z-axis.
    * @throws IllegalArgumentException if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ} is
    *                                  negative.
    */
   public Box3D(Point3DReadOnly position, Orientation3DReadOnly orientation, double sizeX, double sizeY, double sizeZ)
   {
      this();
      set(position, orientation, sizeX, sizeY, sizeZ);
   }

   /**
    * Creates a new box 3D and initializes its pose and size.
    *
    * @param pose  the position and orientation of this box. Not modified.
    * @param sizeX the size of this box along the x-axis.
    * @param sizeY the size of this box along the y-axis.
    * @param sizeZ the size of this box along the z-axis.
    * @throws IllegalArgumentException if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ} is
    *                                  negative.
    */
   public Box3D(Pose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      this();
      set(pose, sizeX, sizeY, sizeZ);
   }

   /**
    * Creates a new box 3D and initializes its pose and size.
    *
    * @param pose  the position and orientation of this box. Not modified.
    * @param sizeX the size of this box along the x-axis.
    * @param sizeY the size of this box along the y-axis.
    * @param sizeZ the size of this box along the z-axis.
    * @throws IllegalArgumentException if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ} is
    *                                  negative.
    */
   public Box3D(RigidBodyTransformReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      this();
      set(pose, sizeX, sizeY, sizeZ);
   }

   /**
    * Creates a new box 3D identical to {@code other}.
    *
    * @param other the other box to copy. Not modified.
    */
   public Box3D(Box3DReadOnly other)
   {
      this();
      set(other);
   }

   /** {@inheritDoc} */
   @Override
   public Shape3DPose getPose()
   {
      return pose;
   }

   /** {@inheritDoc} */
   @Override
   public Vector3DBasics getSize()
   {
      return size;
   }

   /** {@inheritDoc} */
   @Override
   public IntermediateVariableSupplier getIntermediateVariableSupplier()
   {
      return supplier;
   }

   /** {@inheritDoc} */
   @Override
   public void setIntermediateVariableSupplier(IntermediateVariableSupplier newSupplier)
   {
      supplier = newSupplier;
   }

   /**
    * Copies the {@code other} box data into {@code this}.
    *
    * @param other the other box to copy. Not modified.
    */
   @Override
   public void set(Box3D other)
   {
      Box3DBasics.super.set(other);
   }

   @Override
   public Box3D copy()
   {
      return new Box3D(this);
   }

   /**
    * Notifies the internal listeners that this shape has changed.
    */
   public void notifyChangeListeners()
   {
      for (int i = 0; i < changeListeners.size(); i++)
      {
         changeListeners.get(i).changed();
      }
   }

   /**
    * Tests on a per component basis if {@code other} and {@code this} are equal to an {@code epsilon}.
    *
    * @param other   the other box to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two boxes are equal component-wise, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Box3D other, double epsilon)
   {
      return Box3DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two boxes are geometrically similar.
    *
    * @param other   the box to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two boxes represent the same geometry, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(Box3D other, double epsilon)
   {
      return Box3DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(Box3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Box3DReadOnly)
         return Box3DBasics.super.equals((Box3DReadOnly) object);
      else
         return false;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this box 3D.
    *
    * @return the hash code value for this box 3D.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(pose, size);
   }

   /**
    * Provides a {@code String} representation of this box 3D as follows:
    *
    * <pre>
    * Box 3D: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), size: ( 0.191,  0.719,  0.479 )]
    * </pre>
    *
    * @return the {@code String} representing this box 3D.
    */
   @Override
   public String toString()
   {
      return EuclidShapeIOTools.getBox3DString(this);
   }
}
