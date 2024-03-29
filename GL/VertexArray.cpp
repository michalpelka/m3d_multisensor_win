#include "VertexArray.h"
#include "VertexBufferLayout.h"
#include "Renderer.h"
#include <iostream>

VertexArray::VertexArray()
{
	std::cout << "glGenVertexArrays" << glGenVertexArrays << std::endl;
	GLCall(glGenVertexArrays(1, &m_RenderId));
	GLCall(glBindVertexArray(m_RenderId));
}


VertexArray::~VertexArray()
{
	GLCall(glDeleteVertexArrays(1, &m_RenderId));
}

void VertexArray::AddBuffer(const VertexBuffer &vb, const VertexBufferLayout& layout)
{
    Unbind();
	Bind();
	vb.Bind();
	const auto& elements = layout.GetElements();
	unsigned int offset{ 0 };
	for (unsigned int i = 0; i<elements.size(); i++)
	{
		const auto & element = elements[i];
		GLCall(glEnableVertexAttribArray(i));
		GLCall(glVertexAttribPointer(i, element.count, element.type, element.normalized, layout.GetStride(), (const void *)offset));
		offset += element.count* VertexBufferElement::GetSizeOfType(element.type);
	}

}

void VertexArray::Bind() const
{
	GLCall(glBindVertexArray(m_RenderId));
}
void VertexArray::Unbind() const
{
	GLCall(glBindVertexArray(0));
}
