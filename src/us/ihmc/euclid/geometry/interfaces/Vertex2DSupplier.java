package us.ihmc.euclid.geometry.interfaces;

import java.util.Arrays;
import java.util.List;
import java.util.StringJoiner;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

/**
 * Implement this interface to create a custom supplier of 2D vertices or use the static methods to
 * create default suppliers.
 * 
 * @author Sylvain Bertrand
 */
public interface Vertex2DSupplier
{
   /**
    * A supplier with no vertices.
    */
   static final Vertex2DSupplier EMPTY_SUPPLIER = new Vertex2DSupplier()
   {
      @Override
      public Point2DReadOnly getVertex(int index)
      {
         return null;
      }

      @Override
      public int getNumberOfVertices()
      {
         return 0;
      }
   };

   /**
    * Gets the vertex corresponding to the given index.
    * 
    * @param index the index of the vertex, {@code index} &in; [0, {@code getNumberOfVertices()}[.
    * @return the vertex.
    */
   Point2DReadOnly getVertex(int index);

   /**
    * Gets the number of vertices that this supplier holds.
    * 
    * @return the number of vertices.
    */
   public int getNumberOfVertices();

   /**
    * Tests on a per-vertex basis if this supplier and {@code other} are equal.
    * 
    * @param other the other supplier to compare against this.
    * @return {@code true} if the two suppliers are equal.
    */
   default boolean equals(Vertex2DSupplier other)
   {
      if (other == null)
         return false;
      if (getNumberOfVertices() != other.getNumberOfVertices())
         return false;
      for (int i = 0; i < getNumberOfVertices(); i++)
      {
         if (!getVertex(i).equals(other.getVertex(i)))
            return false;
      }
      return true;
   }

   /**
    * Tests on a per-vertex basis if this supplier and {@code other} are equal to an
    * {@code epsilon}.
    * 
    * @param other the other supplier to compare against this.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two suppliers are equal.
    */
   default boolean epsilonEquals(Vertex2DSupplier other, double epsilon)
   {
      if (getNumberOfVertices() != other.getNumberOfVertices())
         return false;
      for (int i = 0; i < getNumberOfVertices(); i++)
      {
         if (!getVertex(i).epsilonEquals(other.getVertex(i), epsilon))
            return false;
      }
      return true;
   }

   /**
    * Gets an empty supplier.
    * 
    * @return the supplier.
    */
   public static Vertex2DSupplier emptyVertex2DSupplier()
   {
      return EMPTY_SUPPLIER;
   }

   /**
    * Returns a fixed-size supplier backed by the given array.
    * 
    * @param vertices the array by which the supplier will be backed.
    * @return the supplier.
    */
   public static Vertex2DSupplier asVertex2DSupplier(Point2DReadOnly... vertices)
   {
      return asVertex2DSupplier(Arrays.asList(vertices));
   }

   /**
    * Returns a fixed-size supplier backed by a portion of the given array starting with the first
    * element and specified by its length {@code numberOfVertices}.
    * 
    * @param vertices the array by which the supplier will be backed.
    * @param numberOfVertices the portion's length.
    * @return the supplier.
    */
   public static Vertex2DSupplier asVertex2DSupplier(Point2DReadOnly[] vertices, int numberOfVertices)
   {
      return asVertex2DSupplier(Arrays.asList(vertices), numberOfVertices);
   }

   /**
    * Returns a fixed-size supplier backed by a portion of the given array specified by the first
    * index {@code startIndex} and the portion length {@code numberOfVertices}.
    * 
    * @param vertices the array by which the supplier will be backed.
    * @param startIndex the first vertex index.
    * @param numberOfVertices the portion's length.
    * @return the supplier.
    */
   public static Vertex2DSupplier asVertex2DSupplier(Point2DReadOnly[] vertices, int startIndex, int numberOfVertices)
   {
      if (numberOfVertices == 0)
         return emptyVertex2DSupplier();
      if (startIndex + numberOfVertices > vertices.length)
         throw new IllegalArgumentException("The array is too small. Array length = " + vertices.length + ", expected minimum length = "
               + (startIndex + numberOfVertices));

      return asVertex2DSupplier(Arrays.asList(vertices), startIndex, numberOfVertices);
   }

   /**
    * Returns a fixed-size supplier backed by the given list.
    * 
    * @param vertices the list by which the supplier will be backed.
    * @return the supplier.
    */
   public static Vertex2DSupplier asVertex2DSupplier(List<? extends Point2DReadOnly> vertices)
   {
      return asVertex2DSupplier(vertices, vertices.size());
   }

   /**
    * Returns a fixed-size supplier backed by a portion of the given list starting with the first
    * element and specified by its length {@code numberOfVertices}.
    * 
    * @param vertices the list by which the supplier will be backed.
    * @param numberOfVertices the portion's length.
    * @return the supplier.
    */
   public static Vertex2DSupplier asVertex2DSupplier(List<? extends Point2DReadOnly> vertices, int numberOfVertices)
   {
      return asVertex2DSupplier(vertices, 0, numberOfVertices);
   }

   /**
    * Returns a fixed-size supplier backed by a portion of the given list specified by the first
    * index {@code startIndex} and the portion length {@code numberOfVertices}.
    * 
    * @param vertices the list by which the supplier will be backed.
    * @param startIndex the first vertex index.
    * @param numberOfVertices the portion's length.
    * @return the supplier.
    */
   public static Vertex2DSupplier asVertex2DSupplier(List<? extends Point2DReadOnly> vertices, int startIndex, int numberOfVertices)
   {
      if (numberOfVertices == 0)
         return emptyVertex2DSupplier();
      if (startIndex + numberOfVertices > vertices.size())
         throw new IllegalArgumentException("The list is too small. List size = " + vertices.size() + ", expected minimum size = "
               + (startIndex + numberOfVertices));

      return new Vertex2DSupplier()
      {
         @Override
         public Point2DReadOnly getVertex(int index)
         {
            return vertices.get(index + startIndex);
         }

         @Override
         public int getNumberOfVertices()
         {
            return numberOfVertices;
         }

         @Override
         public String toString()
         {
            return "Vertex 2D Supplier: " + vertices.subList(startIndex, startIndex + numberOfVertices).toString();
         }
      };
   }

   /**
    * Returns a fixed-size supplier backed by a the given array.
    * 
    * @param vertices the array containing the vertices. Each row contains one point whereas the (at
    *           least) two columns contains in order the coordinates x and y. Not modified.
    * @return the supplier.
    */
   public static Vertex2DSupplier asVertex2DSupplier(double[][] vertices)
   {
      return asVertex2DSupplier(vertices, vertices.length);
   }

   /**
    * Returns a fixed-size supplier backed by a portion of the given array starting with the first
    * row and specified by its length {@code numberOfVertices}.
    * 
    * @param vertices the array containing the vertices. Each row contains one point whereas the (at
    *           least) two columns contains in order the coordinates x and y. Not modified.
    * @param numberOfVertices the portion's length.
    * @return the supplier.
    */
   public static Vertex2DSupplier asVertex2DSupplier(double[][] vertices, int numberOfVertices)
   {
      return asVertex2DSupplier(vertices, 0, numberOfVertices);
   }

   /**
    * Returns a fixed-size supplier backed by a portion of the given array specified by the first
    * index {@code startIndex} and the portion length {@code numberOfVertices}.
    * 
    * @param vertices the array containing the vertices. Each row contains one point whereas the (at
    *           least) two columns contains in order the coordinates x and y. Not modified.
    * @param startIndex the first vertex index.
    * @param numberOfVertices the portion's length.
    * @return the supplier.
    */
   public static Vertex2DSupplier asVertex2DSupplier(double[][] vertices, int startIndex, int numberOfVertices)
   {
      if (numberOfVertices == 0)
         return emptyVertex2DSupplier();
      if (startIndex + numberOfVertices > vertices.length)
         throw new IllegalArgumentException("The array is too small. Array length = " + vertices.length + ", expected minimum length = "
               + (startIndex + numberOfVertices));

      return new Vertex2DSupplier()
      {
         @Override
         public Point2DReadOnly getVertex(int index)
         {
            return new Point2DReadOnly()
            {
               @Override
               public double getX()
               {
                  return vertices[startIndex + index][0];
               }

               @Override
               public double getY()
               {
                  return vertices[startIndex + index][1];
               }

               @Override
               public String toString()
               {
                  return EuclidCoreIOTools.getTuple2DString(this);
               }
            };
         }

         @Override
         public int getNumberOfVertices()
         {
            return numberOfVertices;
         }

         @Override
         public String toString()
         {
            StringJoiner joiner = new StringJoiner("[", ", ", "]");
            Arrays.stream(vertices).forEach(v -> joiner.add(EuclidCoreIOTools.getStringOf("(", ")", ", ", v[0], v[1])));
            return "Vertex 2D Supplier: " + joiner;
         }
      };
   }
}
