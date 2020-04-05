package us.ihmc.euclid.referenceFrame.api;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;

import us.ihmc.euclid.tools.EuclidCoreIOTools;

/**
 * Convenience class for defining a mutable signature that can be used to retrieve a {@link Method}.
 * <p>
 * This interface is part of the API testing framework.
 * </p>
 *
 * @see EuclidFrameAPITester
 * @author Sylvain Bertrand
 */
public class MethodSignature
{
   /** The method's name. */
   private String name;
   /** The list of parameter type of the method. */
   private List<Class<?>> parameterTypes;
   /** The method's return type */
   private Class<?> returnType;

   /**
    * Creates an empty signature.
    * <p>
    * This signature needs to be attributed at a name to represent a valid method.
    * </p>
    */
   public MethodSignature()
   {
   }

   /**
    * Creates a new signature and initialize it with the given name and parameter types.
    *
    * @param name           the method's name.
    * @param parameterTypes the method's parameter types.
    */
   public MethodSignature(String name, Class<?>... parameterTypes)
   {
      setName(name);
      setParameterTypes(parameterTypes);
   }

   /**
    * Copy constructor.
    *
    * @param other the original signature to copy. Not modified.
    */
   public MethodSignature(MethodSignature other)
   {
      set(other);
   }

   /**
    * Creates a new signature and initializes it from the given method.
    *
    * @param method the method to get the signature from.
    */
   public MethodSignature(Method method)
   {
      set(method);
   }

   /**
    * Setter that performs a deep copy of the given signature.
    *
    * @param other the signature to copy. Not modified.
    */
   public void set(MethodSignature other)
   {
      name = other.getName();
      setParameterTypes(other.toParameterTypeArray());
      returnType = other.getReturnType();
   }

   /**
    * Sets this signature from the given method.
    *
    * @param method the method to get the signature from.
    */
   public void set(Method method)
   {
      name = method.getName();
      setParameterTypes(method.getParameterTypes());
      returnType = method.getReturnType();
   }

   /**
    * Sets the method's name.
    *
    * @param name the name of the method.
    */
   public void setName(String name)
   {
      this.name = name;
   }

   /**
    * Returns the number of parameters from this signature.
    *
    * @return the number of parameters.
    */
   public int getParameterCount()
   {
      return parameterTypes.size();
   }

   /**
    * Sets this signature's parameter types.
    *
    * @param parameterTypes the parameter types for this signature.
    */
   public void setParameterTypes(Class<?>... parameterTypes)
   {
      this.parameterTypes = new ArrayList<>(Arrays.asList(parameterTypes));
   }

   /**
    * Inserts a parameter type at the given position.
    * <p>
    * The subsequent parameter types are shifted.
    * </p>
    *
    * @param index         the insertion point of the new parameter type.
    * @param parameterType the parameter type to be added to this signature.
    */
   public void addParameterType(int index, Class<?> parameterType)
   {
      parameterTypes.add(index, parameterType);
   }

   /**
    * Removes the parameter type at the given position from this signature.
    *
    * @param index position of the parameter type to be removed.
    * @return the parameter type that was removed.
    */
   public Class<?> removeParameterType(int index)
   {
      return parameterTypes.remove(index);
   }

   /**
    * Replaces the parameter type at the given position.
    *
    * @param index         position of the parameter type to be replaced.
    * @param parameterType the new parameter type.
    * @return the parameter type that was removed.
    */
   public Class<?> setParameterType(int index, Class<?> parameterType)
   {
      return parameterTypes.set(index, parameterType);
   }

   /**
    * Sets the return type of this signature.
    *
    * @param returnType the new return type.
    * @return the previous return type.
    */
   public Class<?> setReturnType(Class<?> returnType)
   {
      Class<?> oldReturnType = this.returnType;
      this.returnType = returnType;
      return oldReturnType;
   }

   /**
    * Returns the method's name.
    *
    * @return the name of the method.
    */
   public String getName()
   {
      return name;
   }

   /**
    * Gets the parameter types of this signature as an array.
    *
    * @return the array of this signature's parameter types.
    */
   public Class<?>[] toParameterTypeArray()
   {
      return parameterTypes.toArray(new Class<?>[0]);
   }

   /**
    * Returns the internal list of parameter types of this signature.
    *
    * @return the list of parameter types.
    */
   public List<Class<?>> getParameterTypes()
   {
      return parameterTypes;
   }

   /**
    * Gets the parameter type at the given position.
    *
    * @param index position of the parameter type to get.
    * @return the parameter type.
    */
   public Class<?> getParameterType(int index)
   {
      return parameterTypes.get(index);
   }

   /**
    * Gets the return type of this signature.
    *
    * @return this signature's return type.
    */
   public Class<?> getReturnType()
   {
      return returnType;
   }

   /**
    * Returns a string representation of this signature, using the types' simple name.
    *
    * @return the string representation of this signature.
    */
   public String getMethodSimpleName()
   {
      return getMethodSimpleName(returnType, name, parameterTypes);
   }

   /**
    * Creates a string representation of the given method using the parameter types' simple name.
    *
    * @param method the method to get the string representation of.
    * @return the string representation of the method.
    */
   public static String getMethodSimpleName(Method method)
   {
      return getMethodSimpleName(method.getReturnType(), method.getName(), method.getParameterTypes());
   }

   /**
    * Creates a string representation for the given method signature using the return and parameter
    * types' simple name.
    *
    * @param returnType     the return type of the method.
    * @param methodName     the name of the method.
    * @param parameterTypes the parameter types of the method.
    * @return the string representation of the method signature.
    */
   public static String getMethodSimpleName(Class<?> returnType, String methodName, Class<?>... parameterTypes)
   {
      return getMethodSimpleName(returnType, methodName, Arrays.asList(parameterTypes));
   }

   /**
    * Creates a string representation for the given method signature using the return and parameter
    * types' simple name.
    *
    * @param returnType     the return type of the method.
    * @param methodName     the name of the method.
    * @param parameterTypes the parameter types of the method.
    * @return the string representation of the method signature.
    */
   public static String getMethodSimpleName(Class<?> returnType, String methodName, Collection<Class<?>> parameterTypes)
   {
      String returnTypeName = returnType == null ? "void" : returnType.getSimpleName();
      return returnTypeName + " " + methodName + EuclidCoreIOTools.getCollectionString("(", ")", ", ", parameterTypes, Class::getSimpleName);
   }

   /**
    * @see #getMethodSimpleName()
    */
   @Override
   public String toString()
   {
      return getMethodSimpleName();
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof MethodSignature)
      {
         MethodSignature other = (MethodSignature) object;

         if (name == null ? other.name != null : !name.equals(other.name))
            return false;
         if (parameterTypes == null ? other.parameterTypes != null : !parameterTypes.equals(other.parameterTypes))
            return false;
         if (returnType != other.returnType)
            return false;

         return true;
      }
      else
      {
         return false;
      }
   }
}
