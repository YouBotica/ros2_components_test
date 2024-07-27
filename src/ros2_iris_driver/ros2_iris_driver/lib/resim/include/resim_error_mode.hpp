#ifndef RESIM_ERROR_MODE__HPP
#define RESIM_ERROR_MODE__HPP

/**
 * @brief Control error behavior or resim libraries
 */
enum ReSimErrorMode
{
	SILENT, ///< Silently recover from errors. Results undefined, library remains in a consistent state.
	PRINT,  ///< Print warning and errors to std::cout.
	THROW,  ///< Throw exceptions to notify of errors in the library.
};

#endif // RESIM_ERROR_MODE__HPP
