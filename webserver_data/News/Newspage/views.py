#from django.http import HttpResponse
from django.shortcuts import render
from .models import posts

def newspage(request):
	entries = posts.objects.all()[:100]
	return render(request, "newspage.html",{ 'posts' : entries })

